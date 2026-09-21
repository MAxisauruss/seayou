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
        log.info("Battery guard reset - latch cleared.")

    # -- the decision ------------------------------------------------------
    def update(self, battery: dict | None, now: float | None = None) -> str:
        """Feed one telemetry sample. Returns the current level.

        `battery` is telemetry["battery"], or None when no sensor is fitted.
        The level never goes backwards - see point 2 in the module docstring.
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

        # ---- which thresholds are breached, and for how long
        candidate = "ok"
        for name, limit in (("land_now", LAND_V), ("return", RETURN_V), ("warn", WARN_V)):
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
        }
