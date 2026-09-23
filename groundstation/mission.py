"""
Fly-to-coordinate guidance for the SeaYou drone.

Turns "go to this latitude/longitude and hold this height" into the same
roll/pitch/throttle bytes a human pilot's sticks produce. Nothing else in
the stack changes: the Pico still runs the fast attitude and rate loops,
and this only replaces the sticks.

--------------------------------------------------------------------------
READ THIS BEFORE ARMING A MISSION
--------------------------------------------------------------------------
This has NEVER been flown. It has been developed and tested only against
sim_drone.py. Two things about the real aircraft make it genuinely
dangerous until proven on a tether:

  * The real GPS module reports 0 satellites and has never produced a fix.
    Without a fix this module refuses to run at all, which is the correct
    behaviour but also means it has never seen real position data.
  * The barometer resolves to about 0.98 m, so height hold will bob by
    roughly a metre no matter how good the control law is. It is adequate
    for "climb to roughly 5 m and loiter"; it is not precision altitude.

The project's own flight notes warn against automating throttle until the
attitude loop is confirmed stable, and the aircraft crashed on 2026-09-12.
Fly this tethered, low, and with a finger on disarm.

--------------------------------------------------------------------------
Why guidance runs here and not on the Pico
--------------------------------------------------------------------------
Position control is SLOW - GPS updates around 10 Hz and a drone takes
seconds to cross open ground - so a few tens of milliseconds of network
latency is irrelevant to it. The fast loops that genuinely cannot tolerate
latency (attitude, rate) stay on the Pico where they always were. This is
the standard split, and it keeps flight-critical firmware untouched.

The consequence to understand: if the ground station link drops mid
mission, the Pi's own failsafe sends throttle 0 after 0.5 s. The drone does
NOT continue the mission on its own. Moving guidance onto the Pi would
change that, and is the next step for true autonomy.
"""
import logging
import math
import time

log = logging.getLogger(__name__)

# --- limits. Every one of these is a safety bound, not a tuning knob. ---

#: Largest tilt the guidance may ever command, degrees. Deliberately far
#: below what a pilot can command by hand: an autonomous loop with a bug
#: should nudge, never throw the aircraft.
MAX_TILT_DEG = 8.0

#: Ground speed the guidance aims for at most, m/s.
MAX_SPEED_MS = 3.0

#: How far out a target may be, metres. A typo in a coordinate is the
#: classic way to send a drone over the horizon; this makes that refusal
#: automatic rather than a matter of noticing.
MAX_RANGE_M = 300.0

#: Height band the guidance will accept, metres above the link-up baseline.
MIN_ALT_M = 1.0
MAX_ALT_M = 30.0

#: Considered "arrived" inside this radius, metres. Below roughly this,
#: consumer GPS noise is larger than the error being corrected.
ARRIVE_RADIUS_M = 2.5

#: Throttle that roughly hovers. Matched to the simulator; the real value
#: must be measured on the aircraft before any real flight.
HOVER_THROTTLE = 45.0

#: Throttle authority the altitude hold's PROPORTIONAL term may add or
#: remove.
THROTTLE_SPAN = 25.0

#: Proportional gain of the height hold, throttle units per metre. Was an
#: unnamed 6.0 in two places; named because the integral gain below is
#: chosen relative to it.
ALT_P_GAIN = 6.0

# --- the integral trim -------------------------------------------------
#
# WHY THIS EXISTS. The height hold used to be pure proportional, and a P
# controller only produces a correction while an error exists. At a steady
# hover the commanded throttle must equal the aircraft's TRUE hover
# throttle, so the error settles at
#
#     err = (true_hover - HOVER_THROTTLE) / ALT_P_GAIN
#
# and stays there. Measured on the bench with the simulator hovering at 60
# against the assumed 45: a 5 m takeoff parked at exactly 2.50 m, throttle
# pinned at 60, never reached the arrive band, and timed out. (60-45)/6 =
# 2.5. It corrected, but to the wrong height.
#
# The integral accumulates that persistent error and walks the bias to
# where it should have been, which demotes HOVER_THROTTLE from a constant
# that must be right to a starting guess. Measure it anyway: the trim can
# only correct a hover the aircraft can actually reach, so a guess too low
# to leave the ground is still a guess too low to leave the ground.

#: Integral gain, throttle units per metre-second.
#:
#: Against the simulator's vertical response this puts the damping ratio
#: near 0.73 - comfortably damped, no overshoot worth the name - while
#: converging in tens of seconds rather than minutes. It was half this to
#: begin with, which was stable but so slow the takeoff timed out before
#: the trim had finished learning.
#:
#: The real aircraft's vertical gain is NOT known, so treat this as a
#: starting point and watch the first tethered hover for hunting.
ALT_I_GAIN = 1.0

#: Do not integrate outside this much error - a sanity guard against
#: learning from a nonsense setpoint, not a transient filter. Ramps are
#: already excluded by the `trim` flag, which only allows learning while
#: the setpoint is stationary.
#:
#: It MUST stay comfortably above the largest error the trim exists to
#: remove, which is ALT_I_LIMIT / ALT_P_GAIN = 3.3 m. It was 2.0, below
#: that, and the result was a trim that refused to learn precisely the
#: biases it was built for: a 15-unit hover error settles at 2.5 m, the
#: band rejected it, and the trim sat at 0.0 while the aircraft parked
#: 2.5 m low and timed out.
ALT_I_BAND_M = 5.0

#: Clamp on the learned trim, throttle units. Covers a true hover anywhere
#: in roughly 25-65 against an assumed 45.
ALT_I_LIMIT = 20.0

#: Only learn while genuinely airborne, metres. On the ground - or held on
#: a taut tether - the error is real, persistent, and NOT a hover bias, so
#: integrating it winds the throttle up and hands the aircraft a surprise
#: the moment it comes free. This is the condition that makes the trim
#: safe to leave running.
ALT_I_MIN_HEIGHT_M = 0.9

#: Hard ceiling on any throttle the guidance may command, percent. The
#: trim can add ALT_I_LIMIT on top of the proportional term, and an
#: integrator with no ceiling is an aircraft accelerating into the sky.
#: Anything needing more than this to hover is too heavy to be flying a
#: mission.
MAX_AUTO_THROTTLE = 85.0

#: A fix older than this is not a fix, seconds.
GPS_TIMEOUT_S = 2.0

# --- position hold ---------------------------------------------------------
#
# The aircraft holds its POSITION whenever it has satellites good enough to
# believe, and falls back to holding only its HEIGHT when it does not. It
# never cuts the motors because the GPS went away.
#
# That last point is the one that matters. Losing satellites used to abort
# with throttle 0 after two seconds - and for those two seconds step()
# returned None, which hands control to whatever the pilot link is sending:
# SAFE_CONTROL, throttle 0, if nobody is flying. A GPS dropout at height is
# a reason to stop trusting position, not a reason to stop flying. The
# barometer still works, and the aircraft can hold a level hover on it
# exactly as it does after every takeoff.

#: Satellites needed before a position is trusted enough to hold. Four is
#: the least that gives a 3D fix at all - measured on this airframe's
#: u-blox 7, outdoors: 4 used, HDOP 2.1, ~4 m of wander sitting still.
HOLD_MIN_SATS = 4

#: Horizontal dilution of precision above which the fix is too poor to hold
#: against: past about 3 the position wanders further than the correction
#: it would drive, and the aircraft chases noise.
HOLD_MAX_HDOP = 3.0

#: With no usable fix for this long while it should be holding position,
#: come down under control rather than hovering blind while the wind takes
#: it. Long enough to ride out a dropout under a bridge or beside a
#: building; short enough that it cannot drift far.
BLIND_HOVER_LAND_S = 20.0

#: Below this height the aircraft is kept LEVEL even when it has a fix.
#: Leaning to correct position with a leg or prop tip still near the
#: ground is how a quad catches an edge and flips on takeoff or touchdown.
#: 0.8 m is the barometer's first step (it resolves ~0.83 m), so the hold
#: engages at the first reading that proves the aircraft is clear - a 1.0 m
#: gate waited a whole extra step, ~2 s of drifting in a slow climb.
HOLD_MIN_HEIGHT_M = 0.8

#: A landing never TRAVELS. If the position it is holding is further away
#: than this - it drifted while blind, or the fix jumped - it re-anchors
#: to where it is now and comes straight down, instead of flying sideways
#: while it descends.
LAND_HOLD_RECAPTURE_M = 5.0

#: The hold leans harder the further it is pushed off its spot, and hits
#: MAX_TILT_DEG at MAX_SPEED_MS / 0.5 = 6 m out. Past that it has nothing
#: left: if it is STILL losing ground there, the wind is stronger than the
#: aircraft is allowed to fight. Measured in the simulator: a 4 m/s wind
#: carried it away at ~1 m/s while the status went on saying "holding".
OVERPOWER_DIST_M = MAX_SPEED_MS / 0.5

#: How fast it must still be losing ground at full lean, over the last
#: OVERPOWER_WINDOW_S, before that is called being overpowered rather than
#: a gust it will recover from.
OVERPOWER_RATE_MS = 0.2
OVERPOWER_WINDOW_S = 6.0

#: Abort a mission that has run this long without arriving, seconds.
MISSION_TIMEOUT_S = 120.0

# --- takeoff and landing ----------------------------------------------
#
# THE BAROMETER IS THE LIMIT HERE, and it is worth knowing the number
# before trusting any of this. The firmware sends pressure as
# (uint16)(pascals / 10), so the smallest change that can be transmitted
# is 10 Pa - which is 0.83 m of height. Not the sensor's fault: a BMP388
# resolves far better than that, and the resolution is lost in the packet.
#
# So height is known to about a metre, and every constant below is sized
# for that. Anything tighter would be measuring noise.

#: Climb rate the takeoff aims for, m/s. Deliberately slow: the height
#: it is climbing against only updates every 0.83 m, so a fast climb
#: would overshoot by most of a step before the controller noticed.
CLIMB_RATE_MS = 0.5

#: Descent rate the landing aims for, m/s. Slower than the climb, because
#: the ground does not move out of the way.
DESCENT_RATE_MS = 0.4

#: Counted as "at the commanded height". One barometer step is 0.83 m, so
#: this is already as tight as the hardware allows.
ALT_ARRIVE_M = 1.0

#: Below this the landing stops trying to measure height and just ramps
#: the throttle down. Within one step of the ground the barometer cannot
#: tell 0.8 m from touchdown, so continuing to trust it is how you cut
#: the motors while still in the air.
LAND_CUTOFF_M = 0.9

#: How long the final throttle ramp takes, seconds. A ramp rather than a
#: cut: from one barometer step up, a cut IS a drop.
LAND_RAMP_S = 3.0

#: Time a takeoff gets ON TOP OF the climb itself, seconds.
#:
#: The allowance is (target height / CLIMB_RATE_MS) + this, so a 10 m
#: climb is not held to the same clock as a 2 m one. It was a flat 30 s,
#: which is less than the 20 s ramp plus settling that a 10 m takeoff
#: needs - the same mistake as a flat mission timeout against a
#: variable-distance waypoint, and it bit for the same reason: the
#: manoeuvre's duration depends on how far it has to go.
TAKEOFF_MARGIN_S = 30.0

#: After this a landing stops descending and starts the ramp regardless,
#: on the assumption it is near the ground and the barometer has drifted.
LAND_TIMEOUT_S = 60.0

#: Highest a takeoff may be commanded to, metres. Far below MAX_ALT_M on
#: purpose: this feature has never been flown, and the highest this
#: aircraft has ever been is about 3 m.
MAX_TAKEOFF_ALT_M = 10.0

#: A takeoff is refused above this - it is already flying.
ON_GROUND_M = 1.0

M_PER_DEG_LAT = 111_320.0


def haversine_ne(lat1, lon1, lat2, lon2):
    """North/East offset in metres from point 1 to point 2.

    Equirectangular approximation - error is negligible over the few
    hundred metres MAX_RANGE_M allows, and it is far cheaper and easier to
    reason about than the full haversine.
    """
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    north = dlat * M_PER_DEG_LAT
    east = dlon * M_PER_DEG_LAT * math.cos(math.radians((lat1 + lat2) / 2.0))
    return north, east


class Mission:
    """One fly-to-coordinate task and the guidance that executes it.

    States: idle -> running -> (arrived | aborted)
    """

    def __init__(self):
        # idle -> running -> holding (arrived, loitering) -> aborted
        self.state = "idle"
        self.reason = ""
        self.target = None          # (lat, lon)
        # Landing bookkeeping: the height it started down from, and when
        # the final throttle ramp began (None until it does).
        self.land_from_alt = 0.0
        self._land_cut_at = None
        # The learned hover trim, throttle units, and the timestamp the
        # last integration step used. Deliberately NOT reset between
        # manoeuvres: the hover throttle is a property of the aircraft,
        # not of the flight, so what one takeoff learns the next one
        # starts with.
        self._alt_trim = 0.0
        self._alt_last_t = None
        self.target_alt = 0.0
        self.started_at = 0.0
        self.last_fix_at = 0.0
        # Position hold. `position_hold` is what the aircraft is ACTUALLY
        # doing right now - "gps", "height_only" or None - so the dashboard
        # can say "drifting" instead of implying a hold that is not
        # happening. `_blind_since` is when a usable fix was last lost.
        self.position_hold = None
        self._blind_since = None
        self._hold_note = ""
        # (time, distance) while station-keeping, to spot the wind winning.
        self._hold_track = []
        self.distance_m = None
        self.bearing_deg = None
        self.interrupted = False
        # Where "home" is: the last position the aircraft was known to be at
        # while on the ground with a fix. Set by the caller (drone_agent), not
        # inferred here - the mission has no idea what "on the ground" means.
        self.home = None            # (lat, lon) or None
        # Set by return_home(): on arrival, land instead of loitering. A
        # normal waypoint mission loiters and waits for the pilot, which is
        # the wrong ending for a flight that is coming back because the pack
        # is nearly empty.
        self._land_on_arrival = False

    # --- lifecycle -------------------------------------------------
    def start(self, lat, lon, alt, telemetry):
        """Validate and begin. Returns (ok, message).

        Everything that can be checked before moving is checked here, so a
        bad mission is refused on the ground rather than discovered in the
        air.
        """
        if self.state == "running":
            return False, "a mission is already running"
        # Starting a new leg while loitering is normal - that is how you
        # fly a route. It simply retargets.

        gps = (telemetry or {}).get("gps") or {}
        if not gps.get("has_fix") or gps.get("lat") is None or gps.get("lon") is None:
            return False, "no GPS fix - cannot navigate"
        if not (MIN_ALT_M <= alt <= MAX_ALT_M):
            return False, f"height must be between {MIN_ALT_M:g} and {MAX_ALT_M:g} m"
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return False, "coordinates out of range"

        north, east = haversine_ne(gps["lat"], gps["lon"], lat, lon)
        dist = math.hypot(north, east)
        if dist > MAX_RANGE_M:
            return False, f"target is {dist:.0f} m away, limit is {MAX_RANGE_M:.0f} m"

        self.target = (lat, lon)
        self.target_alt = float(alt)
        self._blind_since = None
        self._hold_track = []
        self.state = "running"
        self.reason = ""
        self.started_at = time.monotonic()
        self.last_fix_at = time.monotonic()
        self.distance_m = dist
        self.interrupted = False
        return True, f"mission started - {dist:.0f} m to run"

    def set_home(self, lat, lon):
        """Record where home is. Called while the aircraft is on the ground."""
        if lat is None or lon is None:
            return False
        first = self.home is None
        self.home = (float(lat), float(lon))
        if first:
            log.info("Home set: %.6f, %.6f", self.home[0], self.home[1])
        return True

    def return_home(self, telemetry, alt=None):
        """Fly back to home and land there.

        Falls back to a straight landing whenever going home is not possible -
        no home recorded, no fix, or out of range. That fallback is the normal
        case on this airframe today, and it is the right one: a low pack with
        no navigation still needs to come DOWN, and landing where it is beats
        staying up until the motors stop.

        Returns (ok, message).
        """
        gps = (telemetry or {}).get("gps") or {}
        baro = (telemetry or {}).get("baro") or {}

        if self.home is None:
            ok, msg = self.land(telemetry)
            return ok, ("no home recorded - landing here instead (%s)" % msg
                        if not ok else "no home recorded - landing here")

        if not gps.get("has_fix") or gps.get("lat") is None or gps.get("lon") is None:
            ok, msg = self.land(telemetry)
            return ok, ("no GPS fix - landing here instead (%s)" % msg
                        if not ok else "no GPS fix - landing here")

        north, east = haversine_ne(gps["lat"], gps["lon"], self.home[0], self.home[1])
        dist = math.hypot(north, east)
        if dist > MAX_RANGE_M:
            ok, msg = self.land(telemetry)
            return ok, ("home is %.0f m away, past the %.0f m limit - landing here"
                        % (dist, MAX_RANGE_M))

        # Already there? Just come down.
        if dist <= ARRIVE_RADIUS_M:
            ok, msg = self.land(telemetry)
            return ok, ("already home - landing" if ok else msg)

        # Return at the height it is at now, clamped into the allowed band.
        # Climbing to a fixed transit height on a nearly flat pack would spend
        # the reserve this whole feature exists to protect.
        here = baro.get("height_m")
        want = alt if alt is not None else (here if here is not None else MIN_ALT_M)
        want = max(MIN_ALT_M, min(MAX_ALT_M, float(want)))

        ok, message = self.start(self.home[0], self.home[1], want, telemetry)
        if ok:
            self._land_on_arrival = True
            return True, "returning home - %.0f m to run, landing on arrival" % dist
        # start() refused. Come down where we are rather than staying up.
        ok, msg = self.land(telemetry)
        return ok, ("cannot navigate home (%s) - landing here" % message)

    def hold_here(self, telemetry, reason):
        """Stop travelling and loiter at the CURRENT position.

        This is what a drowning detection triggers. Deliberately NOT abort():
        abort means throttle 0, which drops the aircraft. What is wanted is
        "stop going anywhere, stay exactly here, keep the camera on the
        person" - so the target is retargeted to where the drone already is
        and the existing station-keeping does the rest.

        Returns True if it took effect.
        """
        gps = (telemetry or {}).get("gps") or {}
        if self.state not in ("running", "holding"):
            return False
        if gps.get("lat") is None or gps.get("lon") is None:
            # No position to hold. Falling back to abort would cut the
            # throttle, so leave the mission running and let the normal
            # GPS-loss handling deal with it - that path already knows how
            # to wait GPS_TIMEOUT_S before giving up.
            return False
        self.target = (gps["lat"], gps["lon"])
        self.state = "holding"
        self.reason = reason
        self.interrupted = True
        return True

    def takeoff(self, alt, telemetry):
        """Climb straight up to `alt` and hold there.

        Needs the barometer and nothing else - notably NOT a GPS fix,
        which matters because this aircraft has never had one. That makes
        takeoff and hold the only autonomous thing it can currently do.

        The trade is that nothing holds POSITION: roll and pitch are held
        level, so any wind walks the aircraft sideways while it climbs.
        The pilot has to be ready on the sticks. Returns (ok, message).
        """
        if self.state in ("running", "holding", "takeoff", "landing"):
            return False, "something is already running - abort or land first"

        baro = (telemetry or {}).get("baro") or {}
        alt_now = baro.get("height_m")
        if alt_now is None:
            # An open-loop throttle ramp with no height feedback is how a
            # drone goes through a ceiling. Refuse rather than guess.
            return False, "no barometer - cannot take off to a height"

        try:
            alt = float(alt)
        except (TypeError, ValueError):
            return False, "height must be a number"
        if not (MIN_ALT_M <= alt <= MAX_TAKEOFF_ALT_M):
            return False, ("height must be between %.0f and %.0f m"
                           % (MIN_ALT_M, MAX_TAKEOFF_ALT_M))
        if alt_now > ON_GROUND_M:
            return False, ("already %.1f m up - this is for taking off from "
                           "the ground" % alt_now)

        self.target = None
        self.target_alt = alt
        self.distance_m = None
        self.bearing_deg = None
        self.interrupted = False
        self._land_cut_at = None
        self.position_hold = None
        self._blind_since = None
        self._hold_note = ""
        self._hold_track = []
        # Anchor on the spot it is leaving from, when the sky allows. The
        # climb then holds over it, and so does the hover at the top.
        gps_now = (telemetry or {}).get("gps") or {}
        if self._usable_fix(gps_now)[0]:
            self.target = (gps_now["lat"], gps_now["lon"])
        self.state = "takeoff"
        self.reason = "climbing to %.1f m" % alt
        self.started_at = time.monotonic()
        self.last_fix_at = time.monotonic()
        return True, "taking off to %.1f m" % alt

    def land(self, telemetry):
        """Come down under control and idle the motors.

        Allowed from ANY flying state, because it is the thing you reach
        for when you want whatever is happening to stop safely. That is
        the difference between this and abort(), which idles the motors
        immediately and, from height, drops the aircraft.

        Returns (ok, message).
        """
        baro = (telemetry or {}).get("baro") or {}
        alt_now = baro.get("height_m")
        if alt_now is None:
            return False, "no barometer - cannot land automatically"

        self.land_from_alt = max(0.0, alt_now)
        self.target = None
        self.distance_m = None
        self.bearing_deg = None
        self._land_cut_at = None
        self.state = "landing"
        self.reason = "landing from %.1f m" % alt_now
        self.started_at = time.monotonic()
        self.last_fix_at = time.monotonic()
        return True, "landing from %.1f m" % alt_now

    def clear_return(self):
        """Forget any pending land-on-arrival. Used when a leg is replaced."""
        self._land_on_arrival = False

    def abort(self, reason="aborted by operator"):
        # Includes takeoff and landing, so a stick input still takes the
        # aircraft back instantly from either. Note what that means from a
        # climb: abort idles the motors, so from 2 m it is a 2 m drop. It
        # is still right - the pilot has the sticks the moment it happens
        # - but LAND is the button for coming down on purpose.
        if self.state in ("running", "holding", "takeoff", "landing"):
            self.state = "aborted"
            self.reason = reason
        return self.status()

    def status(self):
        return {
            "state": self.state,
            "reason": self.reason,
            # True when a detection (not the operator) stopped the mission.
            "interrupted_by_detection": self.interrupted,
            "target": {"lat": self.target[0], "lon": self.target[1]} if self.target else None,
            "target_alt_m": self.target_alt,
            "distance_m": round(self.distance_m, 1) if self.distance_m is not None else None,
            "bearing_deg": round(self.bearing_deg, 0) if self.bearing_deg is not None else None,
            "elapsed_s": round(time.monotonic() - self.started_at, 1) if self.started_at else 0,
            # What is actually holding the aircraft still: "gps" (position
            # AND height), "height_only" (it will drift), or None (not
            # hovering). Shown so nobody mistakes a drift for a hold.
            "position_hold": self.position_hold,
            "gps_lost_s": (round(time.monotonic() - self._blind_since, 1)
                           if self._blind_since is not None else None),
            # What the hover trim has learned, and the hover throttle it
            # implies. Worth showing: after one steady hover this is a
            # MEASUREMENT of the aircraft's hover throttle, which is the
            # number HOVER_THROTTLE should have been set to.
            "hover_trim": round(self._alt_trim, 1),
            "hover_learned": round(HOVER_THROTTLE + self._alt_trim, 1),
            "limits": {
                "max_tilt_deg": MAX_TILT_DEG, "max_speed_ms": MAX_SPEED_MS,
                "max_range_m": MAX_RANGE_M, "min_alt_m": MIN_ALT_M, "max_alt_m": MAX_ALT_M,
                "max_takeoff_alt_m": MAX_TAKEOFF_ALT_M,
                # What the height reading is actually worth, so the UI can
                # say so instead of showing 2 decimal places of fiction.
                "baro_step_m": 0.83,
            },
        }

    # --- the guidance loop -----------------------------------------
    @staticmethod
    def _usable_fix(gps):
        """(usable, why_not) - is this fix good enough to hold against?

        HDOP is only checked when the telemetry carries it. The Pi's USB
        GNSS reader does; the Pico's old GPS block did not, and a missing
        number is not evidence of a bad fix.
        """
        if (not gps.get("has_fix") or gps.get("lat") is None
                or gps.get("lon") is None):
            return False, "no GPS fix"
        sats = gps.get("sat_count") or 0
        if sats < HOLD_MIN_SATS:
            return False, "only %d satellite%s" % (sats, "" if sats == 1 else "s")
        hdop = gps.get("hdop")
        if isinstance(hdop, (int, float)) and hdop > HOLD_MAX_HDOP:
            return False, "fix too poor (HDOP %.1f)" % hdop
        return True, ""

    def _blind_hover(self, telemetry, base_control, why):
        """No usable position: hold HEIGHT, level, and wait for it back.

        Keeps the target, so the moment the fix returns the hold resumes
        where it was. After BLIND_HOVER_LAND_S it lands under control
        instead of hovering blind while the wind carries it off.
        """
        now = time.monotonic()
        if self._blind_since is None:
            self._blind_since = now
            log.warning("Position hold suspended - %s. Holding height only.", why)
        self.position_hold = "height_only"
        blind_s = now - self._blind_since
        baro = (telemetry or {}).get("baro") or {}
        alt = baro.get("height_m")

        if blind_s > BLIND_HOVER_LAND_S:
            ok, message = self.land(telemetry)
            if ok:
                self.reason = ("no usable GPS for %.0f s (%s) - landing where "
                               "it is" % (blind_s, why))
                log.warning("GPS gone for %.0f s - landing.", blind_s)
                return self._vertical_step(alt, base_control)
            log.error("GPS gone and cannot land (%s) - holding height.", message)

        self.reason = ("%s - holding height only, DRIFTING. Lands if it is "
                       "not back within %.0f s"
                       % (why, max(0.0, BLIND_HOVER_LAND_S - blind_s)))
        if alt is None:
            return self._abort_control("lost the barometer and the GPS")
        return self._alt_control(self.target_alt, alt, base_control, trim=True)

    @staticmethod
    def _steer(north, east, yaw_deg):
        """Tilt that moves the aircraft towards a north/east offset.

        Returns (roll_deg, pitch_deg). The ONE place position is turned into
        attitude, used by travelling, holding, takeoff and landing alike -
        two copies of a control law are two things to keep in step, and
        the day they drift apart the aircraft flies differently depending
        on which phase it is in.

        Proportional on distance, capped to MAX_SPEED_MS, then turned into
        a tilt. Capping speed before tilt means the aircraft eases off as it
        approaches instead of braking hard.
        """
        dist = math.hypot(north, east)
        speed = max(-MAX_SPEED_MS, min(MAX_SPEED_MS, dist * 0.5))
        if dist > 1e-6:
            want_n = speed * (north / dist)
            want_e = speed * (east / dist)
        else:
            want_n = want_e = 0.0
        yaw = math.radians(yaw_deg)
        fwd = want_n * math.cos(yaw) + want_e * math.sin(yaw)
        rgt = -want_n * math.sin(yaw) + want_e * math.cos(yaw)
        tilt_per_ms = MAX_TILT_DEG / MAX_SPEED_MS
        pitch_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, fwd * tilt_per_ms))
        roll_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, rgt * tilt_per_ms))
        return roll_deg, pitch_deg

    def _hold_during_vertical(self, ctrl, gps, att, alt):
        """Keep the aircraft over one spot while it climbs or descends.

        Takeoff and landing only ever controlled HEIGHT, so in wind the
        aircraft went where the air took it: measured in the simulator, a
        2 m/s breeze carried it 22 m sideways during a 12 s climb, before
        the hover's position hold had even started. The same drift on the
        way down would put a battery return 15-20 m off the home it had
        just flown back to.

        Laid over the vertical controller's output: height stays exactly
        as it was, and roll and pitch are added only when there is a fix
        worth trusting and the aircraft is high enough to lean safely.
        """
        if ctrl is None or self.state not in ("takeoff", "landing"):
            return ctrl
        if ctrl.get("throttle", 0) <= 0:
            return ctrl  # idle or touched down - never lean on the ground
        if alt is None or alt < HOLD_MIN_HEIGHT_M:
            self.position_hold = None
            return ctrl
        usable, _why = self._usable_fix(gps)
        if not usable:
            self.position_hold = "height_only"
            return ctrl

        if self.target is None:
            self.target = (gps["lat"], gps["lon"])
            log.info("Holding position during %s at %.6f, %.6f",
                     self.state, gps["lat"], gps["lon"])
        north, east = haversine_ne(gps["lat"], gps["lon"],
                                   self.target[0], self.target[1])
        if self.state == "landing" and math.hypot(north, east) > LAND_HOLD_RECAPTURE_M:
            self.target = (gps["lat"], gps["lon"])
            north = east = 0.0

        roll_deg, pitch_deg = self._steer(north, east, att.get("yaw", 0.0))
        out = dict(ctrl)
        out["roll"] = _to_byte(roll_deg)
        out["pitch"] = _to_byte(pitch_deg)
        self.position_hold = "gps"
        return out

    def step(self, telemetry, base_control):
        """Return the control dict to send, or None to leave control alone.

        None means "not my turn" - the caller then sends whatever the pilot
        is doing. That is what makes manual input always win: the mission
        contributes nothing unless it is running and healthy.
        """
        # "holding" keeps flying: on arrival the aircraft must LOITER, not
        # be handed back to a safe state that means throttle 0. Returning
        # None here on arrival was a real bug - the drone reached the
        # waypoint and then descended, because nothing was commanding
        # throttle any more.
        if self.state not in ("running", "holding", "takeoff", "landing"):
            return None

        gps = (telemetry or {}).get("gps") or {}
        att = (telemetry or {}).get("attitude") or {}
        baro = (telemetry or {}).get("baro") or {}

        # Takeoff and landing are vertical-only and do not touch any of
        # the GPS logic below - they have to work on an aircraft whose
        # GPS has never produced a fix, which is this one.
        #
        # `holding` with no target is the SAME case: it is what a finished
        # takeoff becomes, a hover at a height with no position to hold.
        # Without that second condition a completed takeoff fell through
        # to the waypoint path, found no fix and aborted itself - at
        # height, with the motors idled, seconds after appearing to work.
        if self.state in ("takeoff", "landing"):
            alt = baro.get("height_m")
            ctrl = self._vertical_step(alt, base_control)
            return self._hold_during_vertical(ctrl, gps, att, alt)

        # A finished takeoff is a hover with no position yet. If the sky is
        # good enough, lock where it is and station-keep from here on;
        # otherwise hold height only, and keep checking - satellites often
        # arrive a few seconds after the props are already turning.
        if self.state == "holding" and self.target is None:
            usable, why = self._usable_fix(gps)
            if not usable:
                self.position_hold = "height_only"
                note = "%s - holding height only, will drift" % why
                if note != self._hold_note:
                    self._hold_note = note
                    self.reason = note
                return self._vertical_step(baro.get("height_m"), base_control)
            self.target = (gps["lat"], gps["lon"])
            self.last_fix_at = time.monotonic()
            self._blind_since = None
            self._hold_note = ""
            self.reason = ("holding position on GPS (%d satellites)"
                           % (gps.get("sat_count") or 0))
            log.info("Position hold engaged at %.6f, %.6f (%d sats)",
                     gps["lat"], gps["lon"], gps.get("sat_count") or 0)
            # ...and fall through into the guidance below, which is what
            # station-keeps against drift.

        # Abort conditions, checked before anything is commanded.
        #
        # The timeout is scoped to travelling. Applying it to a loiter would
        # stop an arrived mission after two minutes, and the way this used to
        # stop things was throttle 0 - the same drop bug in a different place.
        #
        # It no longer aborts at all. A timeout says "this is taking longer
        # than expected", which is NOT the same statement as "what we know
        # about the aircraft is wrong", so it holds position and waits for
        # the pilot rather than cutting the motors. It used to drop the
        # aircraft beside a waypoint it had very nearly reached - a mission
        # that appeared to work right up until it fell out of the sky.
        #
        # The two aborts below are deliberately still aborts: a lost fix and
        # a drift outside the allowed range both mean the POSITION cannot be
        # trusted, and station-keeping against a position you do not believe
        # is worse than handing back with the motors idle.
        if (self.state == "running"
                and time.monotonic() - self.started_at > MISSION_TIMEOUT_S):
            if (gps.get("has_fix") and gps.get("lat") is not None
                    and gps.get("lon") is not None):
                # Retarget to where the drone actually is, so the guidance
                # below station-keeps HERE instead of carrying on towards a
                # waypoint the mission has just given up on. Note this does
                # not set self.interrupted - that flag means a detection
                # stopped the mission, and reporting a timeout as a drowning
                # detection would be a lie to the operator.
                self.target = (gps["lat"], gps["lon"])
                self.state = "holding"
                self.reason = ("timed out before arriving - holding position, "
                               "take over when ready")
            else:
                # No position to hold against. This used to abort with
                # throttle 0 - the same motor cut as a lost fix, reached by
                # a different road. Hold height instead; the blind-hover
                # limit lands it if the sky does not come back.
                return self._blind_hover(telemetry, base_control,
                                         "timed out before arriving, no GPS fix")
        usable, why = self._usable_fix(gps)
        if not usable:
            # This used to be `return None` for two seconds and then an
            # abort with throttle 0 - a motor cut at height, triggered by
            # nothing more than the sky. Now: hold height, keep the
            # target, and resume the moment the fix is back.
            return self._blind_hover(telemetry, base_control, why)
        if self._blind_since is not None:
            log.info("GPS back after %.1f s - position hold resumed.",
                     time.monotonic() - self._blind_since)
            self._blind_since = None
            if self.state == "holding":
                self.reason = ("holding position on GPS (%d satellites)"
                               % (gps.get("sat_count") or 0))
        self.last_fix_at = time.monotonic()
        if self.position_hold != "overpowered":
            self.position_hold = "gps"

        north, east = haversine_ne(gps["lat"], gps["lon"], self.target[0], self.target[1])
        dist = math.hypot(north, east)
        self.distance_m = dist
        self.bearing_deg = (math.degrees(math.atan2(east, north)) + 360) % 360

        if dist > MAX_RANGE_M * 1.5:
            # Either the drone or the target is not where we think it is.
            # That is a reason to stop steering by GPS - and a reason to
            # get down - but not a reason to cut the motors at height,
            # which is what this used to do.
            ok, message = self.land(telemetry)
            if ok:
                self.reason = ("position implausible (%.0f m from target) - "
                               "landing where it is" % dist)
                log.error("Position implausible (%.0f m off) - landing.", dist)
                return self._vertical_step(baro.get("height_m"), base_control)
            return self._abort_control("drifted outside the allowed range")

        alt = baro.get("height_m")
        arrived_alt = alt is None or abs(alt - self.target_alt) < 1.0
        if self.state == "running" and dist <= ARRIVE_RADIUS_M and arrived_alt:
            if self._land_on_arrival:
                # Came back on a low pack. Do not loiter waiting for a pilot
                # who may not be watching - put it on the ground.
                self._land_on_arrival = False
                ok, message = self.land(telemetry)
                if ok:
                    log.warning("Home reached - landing.")
                    return self._vertical_step(alt, base_control)
                log.error("Home reached but cannot land (%s) - holding.", message)
            self.state = "holding"
            self.reason = "arrived - holding position, take over when ready"

        # While holding, the same guidance runs - it just has a target it is
        # already at, so it station-keeps against drift instead of
        # travelling. Altitude hold continues either way.
        roll_deg, pitch_deg = self._steer(north, east, att.get("yaw", 0.0))

        # Is the wind winning? Only a HOLD can be overpowered - a transit
        # is meant to be moving. Reported, not acted on: the aircraft keeps
        # flying and keeps leaning into it. Whether to land where it is
        # (which over the sea means in the sea) or fly it out by hand is a
        # decision for the pilot, not for this loop.
        if self.state == "holding":
            now = time.monotonic()
            self._hold_track.append((now, dist))
            self._hold_track = [(t, d) for (t, d) in self._hold_track
                                if now - t <= OVERPOWER_WINDOW_S]
            t0, d0 = self._hold_track[0]
            span = now - t0
            rate = (dist - d0) / span if span >= OVERPOWER_WINDOW_S * 0.8 else 0.0
            if dist >= OVERPOWER_DIST_M and rate >= OVERPOWER_RATE_MS:
                if self.position_hold != "overpowered":
                    log.error("WIND TOO STRONG: %.0f m off the hold point and "
                              "losing %.1f m/s at full lean.", dist, rate)
                self.position_hold = "overpowered"
                self.reason = ("WIND TOO STRONG TO HOLD - leaning at the %.0f deg "
                               "limit and still being pushed away at %.1f m/s "
                               "(%.0f m off). Take over."
                               % (MAX_TILT_DEG, rate, dist))
            elif self.position_hold == "overpowered" and rate < OVERPOWER_RATE_MS / 2:
                self.position_hold = "gps"
                self.reason = ("holding position on GPS again (%.0f m off the "
                               "hold point)" % dist)
        else:
            self._hold_track = []

        # Vertical: the same height hold takeoff and landing use, so the
        # hover trim learned by one is used by all of them. The waypoint's
        # height setpoint does not move, so this sample is learnable.
        if alt is None:
            throttle = HOVER_THROTTLE + self._alt_trim
        else:
            throttle = self._alt_throttle(self.target_alt, alt, trim=True)
        throttle = max(0.0, min(MAX_AUTO_THROTTLE, throttle))

        ctrl = dict(base_control)
        ctrl["roll"] = _to_byte(roll_deg)
        ctrl["pitch"] = _to_byte(pitch_deg)
        ctrl["yaw"] = 50                      # guidance never commands yaw
        ctrl["throttle"] = int(round(throttle))
        return ctrl

    def _vertical_step(self, alt, base_control):
        """Climb or descend, holding attitude level. Height only."""
        now = time.monotonic()

        if alt is None:
            # The barometer stopped reporting mid-manoeuvre. There is no
            # longer anything to control height against.
            return self._abort_control("lost the barometer")

        # A finished takeoff: hover at the height it was sent to, until a
        # pilot takes over or a landing is commanded. The setpoint is
        # stationary here, so this is where the hover trim is learned.
        if self.state == "holding":
            return self._alt_control(self.target_alt, alt, base_control,
                                     trim=True)

        if self.state == "takeoff":
            allowance = self.target_alt / CLIMB_RATE_MS + TAKEOFF_MARGIN_S
            if now - self.started_at > allowance:
                # LAND, do not abort. Abort means throttle 0, and a takeoff
                # that runs out of time is by definition already in the air
                # - so aborting drops it from whatever height it did reach.
                #
                # This is not hypothetical. With HOVER_THROTTLE set wrong
                # (45 assumed, 60 actual) the climb settles at a permanent
                # steady-state error of (60-45)/6 = 2.5 m below target,
                # never reaches the arrive band, and times out - and the
                # old behaviour then cut the motors at 2.5 m. Observed on
                # the bench, not theorised.
                #
                # Coming down under control is the right answer to "this
                # is not working": the aircraft ends up on the ground
                # either way, and only one of them is a landing.
                self.land_from_alt = max(0.0, alt)
                self._land_cut_at = None
                self.state = "landing"
                self.reason = ("could not reach %.1f m - landing"
                               % self.target_alt)
                self.started_at = now
                return self._alt_control(alt, alt, base_control)
            if alt >= self.target_alt - ALT_ARRIVE_M:
                self.state = "holding"
                self.reason = (
                    ("at %.1f m - holding position on GPS, take over when "
                     "ready" % alt) if self.target is not None else
                    ("at %.1f m - holding, take over when ready" % alt))
                return self._alt_control(self.target_alt, alt, base_control,
                                         trim=True)
            # Ramp the SETPOINT rather than aiming at the final height from
            # the start. Chasing the full error would mean full throttle
            # until the barometer noticed, and it only notices every 0.83 m.
            want = min(self.target_alt, CLIMB_RATE_MS * (now - self.started_at))
            # Once the ramp reaches the target the setpoint stops moving,
            # and whatever error is left is a hover bias worth learning.
            #
            # Gating this on state == "holding" instead - the obvious
            # reading - deadlocks: with the hover throttle wrong the
            # aircraft parks below the arrive band and never becomes
            # "holding", so the trim that would have got it there is never
            # learned. Measured: it sat at 2.50 m for a 5 m command with
            # the trim stuck at 0.0 until the takeoff timed out.
            return self._alt_control(want, alt, base_control,
                                     trim=want >= self.target_alt)

        # --- landing ---------------------------------------------------
        if self._land_cut_at is None:
            near_ground = alt <= LAND_CUTOFF_M
            out_of_time = now - self.started_at > LAND_TIMEOUT_S
            if near_ground or out_of_time:
                self._land_cut_at = now
                self.reason = ("touchdown - easing the motors down"
                               if near_ground else
                               "landing timed out - easing the motors down")
            else:
                want = max(0.0, self.land_from_alt
                           - DESCENT_RATE_MS * (now - self.started_at))
                return self._alt_control(want, alt, base_control)

        # Final ramp. Never a step to zero: from one barometer step up,
        # cutting the motors is indistinguishable from dropping it.
        frac = min(1.0, (now - self._land_cut_at) / LAND_RAMP_S)
        throttle = HOVER_THROTTLE * (1.0 - frac)
        if frac >= 1.0:
            self.state = "idle"
            self.reason = "landed - motors idle"
        return self._level_control(base_control, throttle)

    def _alt_control(self, want_alt, alt, base_control, trim=False):
        """Height hold with roll and pitch held level."""
        return self._level_control(
            base_control, self._alt_throttle(want_alt, alt, trim))

    def _alt_throttle(self, want_alt, alt, trim=False):
        """Throttle for a wanted height. Proportional, plus a slow
        integral trim on the hover point.

        `trim` says whether this sample is worth learning from, and only
        a STATIONARY setpoint is: while the aircraft is climbing or
        descending on a ramp the error is the setpoint moving ahead of it,
        and integrating that is textbook windup - the trim would wind in
        during the climb and have to unwind at the top, overshooting.
        """
        now = time.monotonic()
        # Clamped: a stalled loop must not arrive as one enormous step.
        dt = 0.0 if self._alt_last_t is None else min(now - self._alt_last_t, 0.2)
        self._alt_last_t = now

        err = want_alt - alt
        p = max(-THROTTLE_SPAN, min(THROTTLE_SPAN, err * ALT_P_GAIN))
        throttle = HOVER_THROTTLE + self._alt_trim + p

        learnable = (
            trim
            and dt > 0.0
            and abs(err) <= ALT_I_BAND_M
            # Airborne only. See ALT_I_MIN_HEIGHT_M.
            and alt >= ALT_I_MIN_HEIGHT_M
        )
        if learnable:
            # Conditional anti-windup: stop integrating INTO a limit, but
            # keep integrating back out of one, or the trim can latch at
            # the ceiling and never recover.
            into_top = throttle >= MAX_AUTO_THROTTLE and err > 0
            into_bottom = throttle <= 0.0 and err < 0
            if not into_top and not into_bottom:
                self._alt_trim = max(-ALT_I_LIMIT, min(
                    ALT_I_LIMIT, self._alt_trim + err * ALT_I_GAIN * dt))
                throttle = HOVER_THROTTLE + self._alt_trim + p

        return throttle

    def _level_control(self, base_control, throttle):
        ctrl = dict(base_control)
        ctrl["roll"] = 50
        ctrl["pitch"] = 50
        ctrl["yaw"] = 50
        ctrl["throttle"] = int(round(max(0.0, min(MAX_AUTO_THROTTLE, throttle))))
        return ctrl

    def _abort_control(self, reason):
        self.state = "aborted"
        self.reason = reason
        # Level and throttle zero. Deliberately NOT a hover: an aborted
        # mission means something is wrong with what we know about the
        # aircraft, and holding a hover on bad information is worse than
        # handing control back with the motors idle.
        return {"roll": 50, "pitch": 50, "yaw": 50, "throttle": 0,
                "roll_trim": 25, "pitch_trim": 25, "cmd0": 0, "cmd1": 0}

    def _hold_control(self, base_control):
        ctrl = dict(base_control)
        ctrl["roll"] = 50
        ctrl["pitch"] = 50
        ctrl["yaw"] = 50
        ctrl["throttle"] = int(round(HOVER_THROTTLE))
        return ctrl


def _to_byte(tilt_deg):
    """Tilt in degrees -> the 0-100 wire byte, 50 being level.

    Full deflection is about 30 degrees on this airframe, matching what a
    pilot's stick produces, so guidance and manual input speak the same
    units.
    """
    v = 50 + (tilt_deg / 30.0) * 50.0
    return int(round(max(0, min(100, v))))
