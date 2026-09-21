"""Low-battery failsafe: decide when to come home, and when to come down.

Runs ON THE PI, inside the same 30 Hz loop that drives the Pico, alongside the
rest of the autonomy. It never consults the ground station, so a pack that
goes flat with the network gone is still handled.

This module only DECIDES. It returns a level; drone_agent.py is what acts on
it. Keeping the decision pure is what makes it testable on a bench with no
aircraft attached, which matters for a thing that can put the drone on the
ground on its own.


THE CELLS
---------
Molicel INR-21700-P42A, from the manufacturer's data sheet
(INR21700P42A-01 Rev 0.2), section 4, RATED SPECIFICATIONS:

    Nominal voltage            3.6 V
    End of charge voltage      4.20 +/- 0.05 V
    End of discharge voltage   2.5 V
    Maximum discharge current  45 A   ("cycle life is reduced at high rates")
    Internal resistance        <= 15 mOhm  (AC 1 kHz, fresh cell)
    Rated capacity             4.0 Ah minimum

Note the part number: the "P42A" is 4.2 Ah of CAPACITY, not 42 A of current.
The current rating is 45 A continuous.

These are Li-ion, not LiPo. The discharge curve is different and so is the
floor - 2.5 V/cell, not 3.0 - but 2.5 V is where the cell is empty and further
discharge damages it. It is not a number to fly to. The thresholds below leave
real reserve above it.


WHY VOLTAGE ALONE IS AWKWARD, AND WHAT IS DONE ABOUT IT
-------------------------------------------------------
15 mOhm per cell means the pack sags hard under throttle: 20 A per cell is
0.30 V of sag, 45 A is nearly 0.7 V. A cell resting at 3.7 V reads 3.0 V
during a punch-out. Three consequences, each handled:

  1. A naive instantaneous threshold fires on every aggressive climb.
     -> The voltage is low-pass filtered (FILTER_TAU_S) and the threshold
        must be held continuously for PERSIST_S before anything happens.

  2. Voltage RECOVERS when the throttle comes back. Trigger a return, descend,
     draw less current, and the voltage climbs back over the threshold - which
     would cancel the return, which would climb again, which would trigger it
     again.
     -> Escalation LATCHES. The level only ever moves one way within a flight.
        Nothing but an explicit reset() on the ground clears it.

  3. Cell count cannot be inferred safely from a sagging pack. A 3S at 3.3 V
     per cell and a 4S at 2.5 V per cell are both about 10 V.
     -> PACK_CELLS is explicit, not guessed, and a reading that is impossible
        for that count is rejected rather than acted on.
"""

import logging
import time

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Pack configuration. CHANGE THESE IF THE PACK CHANGES.
# ---------------------------------------------------------------------------

#: Cells in series. Explicit on purpose - see point 3 above. Measured on this
#: airframe 20 Sep 2026: 11.66 V resting, i.e. 3.89 V/cell on a 3S.
PACK_CELLS = 3

#: Data-sheet limits, for sanity checking only - never thresholds to fly to.
CELL_FULL_V = 4.20
CELL_DATASHEET_CUTOFF_V = 2.50

#: A reading outside this per-cell band means the measurement or PACK_CELLS is
#: wrong. The guard reports "unknown" rather than acting on it: a failsafe that
#: lands the aircraft because a sensor glitched is its own hazard.
SANE_MIN_V = 2.00
SANE_MAX_V = 4.35

# ---------------------------------------------------------------------------
# Thresholds, volts per cell, measured UNDER FLIGHT LOAD (so below resting).
#
# Spaced so each level has room to act before the next one matters. On the
# P42A discharge curve these leave roughly: WARN ~30%, RETURN ~20%, LAND ~10%.
# ---------------------------------------------------------------------------

#: Tell the pilot. Nothing automatic happens.
WARN_V = 3.55

#: Come home now. Flies to the recorded home position and lands there; with no
#: GPS fix or no home recorded, lands where it is instead.
RETURN_V = 3.40

#: Down, immediately, wherever it is. Abandons any return in progress - at this
#: point getting on the ground beats getting to a nice spot.
LAND_V = 3.15

# ---------------------------------------------------------------------------
# Filtering
# ---------------------------------------------------------------------------

#: Exponential filter time constant. Long enough to ride through a punch-out,
#: short enough to notice a pack genuinely falling off a cliff.
FILTER_TAU_S = 5.0

#: A threshold must be continuously breached for this long before it counts.
PERSIST_S = 3.0

#: With no usable reading for this long, say so. Deliberately does NOT trigger
#: a landing: an I2C fault is not evidence the battery is flat, and a failsafe
#: that fires on missing data would ground the aircraft every time the bus
#: glitched. It is reported so the pilot can decide.
STALE_AFTER_S = 10.0

# ---------------------------------------------------------------------------
# The return-home reserve
# ---------------------------------------------------------------------------
# RETURN_V on its own is a FIXED threshold, and a fixed threshold cannot
# answer the only question that matters: is there enough left to get back
# from where the aircraft actually is? 3.40 V/cell is generous hovering
# over the launch point and can be far too late 280 m downwind at the far
# corner of a search grid.
#
# So the return threshold moves. The further out the aircraft is, the
# earlier it turns for home - by exactly the voltage the trip home is
# predicted to cost.
#
# HOW THE PREDICTION WORKS, AND WHAT IT IS WORTH
# ----------------------------------------------
# There is no current sensor on this airframe: the INA219 is wired
# voltage-only (battery.py, CURRENT_SENSE_WIRED = False), so the pack's
# remaining energy cannot be measured directly. What CAN be measured is
# how fast the filtered voltage is falling, in volts per second per cell.
# Multiply that by the time the trip home will take and you get the
# voltage it will cost. That is the reserve.
#
# It is an estimate, and it is honest about being one: the safety factor
# below assumes the trip home costs more than the flying done so far
# (wind is usually worse going back, and the descent and landing are not
# free), and the reserve is floored so it can never be smaller than the
# fixed behaviour that existed before.

#: Ground speed the guidance actually flies at, m/s. Must match
#: mission.py's MAX_SPEED_MS - the reserve is computed against the speed
#: the aircraft will really make, not one it cannot reach.
RETURN_CRUISE_MS = 3.0

#: Seconds to add for the descent and landing once it is overhead, plus
#: the turn and acceleration at the start of the run home.
RETURN_OVERHEAD_S = 25.0

#: The trip home is assumed to cost this much more than the flight so far
#: has, per second. Headwind on the way back, and a descent that is not
#: as cheap as it looks once the aircraft has to arrest it.
RETURN_SAFETY_FACTOR = 1.5

#: Never reserve less than this, so a distance-aware guard is never LESS
#: cautious than the fixed threshold it replaced.
MIN_RESERVE_V = 0.0

#: Never reserve more than this. Past here the estimate is telling us the
#: pack is falling off a cliff, and the answer to that is LAND_V, not an
#: ever-growing reserve that grounds the aircraft on the pad.
MAX_RESERVE_V = 0.45

#: Used until enough flight has been seen to measure the real slope.
#: Roughly a 3S pack going 4.15 -> 3.40 V/cell over a ten-minute flight.
DEFAULT_SLOPE_V_PER_S = 0.00125

#: Smoothing for the measured slope. Long, because the number wanted is
#: the trend over a flight, not the dip from one climb.
SLOPE_TAU_S = 45.0

#: Samples closer together than this are ignored for the slope - dividing
#: filter noise by a tiny dt produces a huge fake gradient.
SLOPE_MIN_DT_S = 1.0

#: Ordered worst-last. Used to enforce the latch.
LEVELS = ("unknown", "ok", "warn", "return", "land_now")


def _rank(level: str) -> int:
    try:
        return LEVELS.index(level)
    except ValueError:
        return 0


class BatteryGuard:
    """Turns a stream of pack-voltage readings into a failsafe level."""

    def __init__(self, cells: int = PACK_CELLS):
        self.cells = int(cells)
        self.level = "unknown"
        self.reason = ""
        self.cell_v = None          # filtered, volts per cell
        self.pack_v = None          # filtered, volts
        self._filt = None           # filter state, pack volts
        self._last_t = None
        self._below_since = {}      # threshold name -> monotonic time it went under
        self._last_good_t = None
        self._warned_stale = False
        # Return-reserve state.
        self._slope = None          # V/s per cell, positive = falling
        self._slope_v = None        # last cell voltage used for the slope
        self._slope_t = None
        self._distance_home_m = None
        self._t_home_s = None
        self._reserve_v = 0.0

    # -- lifecycle ---------------------------------------------------------
    def reset(self):
        """Clear the latch. Only safe on the ground, after a pack change."""
        self.level = "unknown"
        self.reason = ""
        self.cell_v = None
        self.pack_v = None
        self._filt = None
        self._last_t = None
        self._below_since = {}
        self._last_good_t = None
        self._warned_stale = False
        self._slope = None
        self._slope_v = None
        self._slope_t = None
        self._distance_home_m = None
        self._t_home_s = None
        self._reserve_v = 0.0
        log.info("Battery guard reset - latch cleared.")

    # -- the decision ------------------------------------------------------
    def update(self, battery: dict | None, now: float | None = None,
               distance_home_m: float | None = None) -> str:
        """Feed one telemetry sample. Returns the current level.

        `battery` is telemetry["battery"], or None when no sensor is fitted.
        The level never goes backwards - see point 2 in the module docstring.

        `distance_home_m` is how far the aircraft is from home right now.
        Pass it and the return threshold rises by the voltage the trip
        home is predicted to cost, so the aircraft turns back while it can
        still get there. Leave it None - no fix, no home - and the guard
        behaves exactly as it did before, on the fixed threshold.
        """
        now = time.monotonic() if now is None else now

        volts = None
        if battery:
            v = battery.get("voltage_v")
            if isinstance(v, (int, float)) and not battery.get("fault"):
                volts = float(v)

        if volts is None:
            # No usable reading. Hold whatever level is latched; do not
            # invent one. Report staleness once it has gone on long enough
            # to be worth mentioning.
            if self._last_good_t is not None and (now - self._last_good_t) > STALE_AFTER_S:
                if not self._warned_stale:
                    log.warning("No pack voltage for %.0f s - battery failsafe is "
                                "flying blind (latched at '%s').",
                                now - self._last_good_t, self.level)
                    self._warned_stale = True
            return self.level

        per_cell_raw = volts / self.cells
        if not (SANE_MIN_V <= per_cell_raw <= SANE_MAX_V):
            # Either the pack is not PACK_CELLS cells, or the reading is
            # junk. Either way, acting on it would be worse than not.
            log.error("Pack voltage %.2f V is %.2f V/cell for a %dS pack - "
                      "outside %.1f-%.1f V. Check PACK_CELLS. Ignoring.",
                      volts, per_cell_raw, self.cells, SANE_MIN_V, SANE_MAX_V)
            return self.level

        self._last_good_t = now
        self._warned_stale = False

        # ---- low-pass filter, proper time-based alpha so an irregular
        # sample rate cannot change the time constant.
        if self._filt is None or self._last_t is None:
            self._filt = volts
        else:
            dt = max(0.0, now - self._last_t)
            alpha = 1.0 - pow(2.718281828459045, -dt / FILTER_TAU_S) if dt > 0 else 0.0
            self._filt += alpha * (volts - self._filt)
        self._last_t = now

        self.pack_v = round(self._filt, 3)
        self.cell_v = round(self._filt / self.cells, 3)

        # ---- how fast the pack is falling, volts per second per cell.
        # Measured off the FILTERED voltage, so a punch-out does not
        # register as the pack collapsing. Only decline counts: voltage
        # recovering when the throttle comes back is not the pack
        # refilling, and letting it drag the estimate down would shrink
        # the reserve exactly when the aircraft is working hardest.
        if self._slope_v is not None and self._slope_t is not None:
            dt = now - self._slope_t
            if dt >= SLOPE_MIN_DT_S:
                fall = (self._slope_v - self.cell_v) / dt
                fall = max(0.0, fall)
                if self._slope is None:
                    self._slope = fall
                else:
                    a = 1.0 - pow(2.718281828459045, -dt / SLOPE_TAU_S)
                    self._slope += a * (fall - self._slope)
                self._slope_v = self.cell_v
                self._slope_t = now
        else:
            self._slope_v = self.cell_v
            self._slope_t = now

        # ---- what the trip home is predicted to cost, in volts per cell
        self._distance_home_m = distance_home_m
        if distance_home_m is None:
            self._t_home_s = None
            self._reserve_v = 0.0
        else:
            self._t_home_s = (max(0.0, float(distance_home_m)) / RETURN_CRUISE_MS
                              + RETURN_OVERHEAD_S)
            slope = DEFAULT_SLOPE_V_PER_S if self._slope is None else max(
                self._slope, DEFAULT_SLOPE_V_PER_S * 0.25)
            self._reserve_v = min(
                MAX_RESERVE_V,
                max(MIN_RESERVE_V, slope * self._t_home_s * RETURN_SAFETY_FACTOR),
            )

        return_limit = RETURN_V + self._reserve_v

        # ---- which thresholds are breached, and for how long
        candidate = "ok"
        for name, limit in (("land_now", LAND_V), ("return", return_limit), ("warn", WARN_V)):
            if self.cell_v <= limit:
                first = self._below_since.get(name)
                if first is None:
                    self._below_since[name] = now
                    first = now
                if (now - first) >= PERSIST_S:
                    candidate = name
                    break
            else:
                self._below_since.pop(name, None)

        # ---- latch: escalate only
        if _rank(candidate) > _rank(self.level) or self.level == "unknown":
            if candidate != self.level:
                self.level = candidate
                self.reason = self._describe(candidate)
                if candidate != "ok":
                    log.warning("BATTERY %s: %.2f V/cell (%.2f V pack) - %s",
                                candidate.upper(), self.cell_v, self.pack_v,
                                self.reason)
        return self.level

    def _describe(self, level: str) -> str:
        if level == "warn":
            return ("pack below %.2f V/cell - finish up and land soon"
                    % WARN_V)
        if level == "return":
            if self._reserve_v > 0.005 and self._distance_home_m is not None:
                return ("pack below %.2f V/cell - that is %.2f V plus a %.2f V "
                        "reserve to fly the %.0f m home - returning now"
                        % (RETURN_V + self._reserve_v, RETURN_V,
                           self._reserve_v, self._distance_home_m))
            return ("pack below %.2f V/cell - returning home" % RETURN_V)
        if level == "land_now":
            return ("pack below %.2f V/cell - landing immediately" % LAND_V)
        return ""

    # -- reporting ---------------------------------------------------------
    def status(self) -> dict:
        """For telemetry, so the dashboard can show what the guard thinks."""
        return {
            "level": self.level,
            "reason": self.reason,
            "cell_v": self.cell_v,
            "pack_v": self.pack_v,
            "cells": self.cells,
            "thresholds": {"warn": WARN_V, "return": RETURN_V, "land_now": LAND_V},
            # What the distance-aware reserve is doing right now. None
            # everywhere means it is not active - no fix, or no home - and
            # the fixed threshold is in charge.
            "reserve": {
                "distance_home_m": (round(self._distance_home_m, 1)
                                    if self._distance_home_m is not None else None),
                "time_home_s": (round(self._t_home_s, 1)
                                if self._t_home_s is not None else None),
                "reserve_v": round(self._reserve_v, 3),
                "return_at_v": round(RETURN_V + self._reserve_v, 3),
                # Volts per cell still in hand before the aircraft turns
                # for home on its own. Negative means it already has.
                "headroom_v": (round(self.cell_v - (RETURN_V + self._reserve_v), 3)
                               if self.cell_v is not None else None),
                # The measured discharge slope, per minute, which is the
                # number a human can sanity-check against the flight.
                "fall_v_per_min": (round(self._slope * 60.0, 4)
                                   if self._slope is not None else None),
                "measured": self._slope is not None,
            },
        }
