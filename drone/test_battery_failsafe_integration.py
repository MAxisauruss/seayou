"""Integration tests: does the failsafe actually command the right thing?

    ssh pi@drone.local 'cd /home/pi/dashboard && python3 test_battery_failsafe_integration.py'

test_battery_guard.py checks the DECISION. This checks the ACTION - that a
flat pack starts a return, that a critical pack overrides it, that neither
happens on the ground, and that the pilot can always take it back.

GroundStationLink is built without __init__ on purpose: the real constructor
opens a serial port and a camera. Only the attributes _battery_failsafe
actually touches are set, so the test exercises the real method against real
Mission code with nothing mocked out underneath it.
"""

import sys
import time

import battery_guard as bg
from battery_guard import BatteryGuard
from drone_agent import GroundStationLink
from mission import Mission

_checks = 0
_fails = 0


def check(ok, what):
    global _checks, _fails
    _checks += 1
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        _fails += 1


def make_link():
    """A GroundStationLink with only what _battery_failsafe uses."""
    link = GroundStationLink.__new__(GroundStationLink)
    link.mission = Mission()
    link._battery_guard = BatteryGuard()
    link._battery_returning = False
    link._battery_override_until = 0.0
    link._battery_no_baro_logged = False
    link._latest_telemetry = {}
    return link


def telem(cell_v, height, fix=True, lat=-25.7485, lon=28.1870, cells=3):
    t = {
        "battery": {"voltage_v": round(cell_v * cells, 3)},
        "baro": {"height_m": height, "pressure_hpa": 870.0, "temp_c": 25},
        "gps": {"has_fix": fix,
                "lat": lat if fix else None,
                "lon": lon if fix else None},
    }
    return t


CENTRED = {"roll": 50, "pitch": 50, "throttle": 0, "yaw": 50}
STICK = {"roll": 90, "pitch": 50, "throttle": 40, "yaw": 50}


def drain(link, cell_v, height, seconds=40, fix=True, control=None,
          link_fresh=True, t0=None, lat=-25.7485):
    """Feed telemetry until the guard settles. Returns the end time."""
    t = t0 if t0 is not None else time.monotonic()
    end = t + seconds
    while t < end:
        link._latest_telemetry = telem(cell_v, height, fix, lat=lat)
        link._battery_guard.update(link._latest_telemetry["battery"], now=t)
        link._battery_failsafe(control or CENTRED, link_fresh, now=t)
        t += 0.5
    return t


# ---------------------------------------------------------------------------
print("\n=== 1. On the ground with a flat pack: do NOT spin the motors ===")
link = make_link()
drain(link, 3.10, height=0.2)        # well below LAND_V, but sitting down
check(link._battery_guard.level == "land_now",
      "the guard does reach 'land_now'")
check(link.mission.state == "idle",
      "but no mission is started on the ground (state stays idle)")

print("\n=== 2. Home is captured while on the ground with a fix ===")
check(link.mission.home == (-25.7485, 28.187),
      "home recorded from the ground position: %s" % (link.mission.home,))

print("\n=== 3. Airborne, pack low: returns home ===")
link = make_link()
drain(link, 3.90, height=0.2)        # on the ground first, to set home
check(link.mission.home is not None, "home is set before takeoff")
link.mission.state = "holding"       # airborne, loitering
drain(link, 3.30, height=8.0, lat=-25.7500)   # drifted ~165 m south
check(link._battery_guard.level in ("return", "land_now"), "guard escalated")
check(link._battery_returning is True, "a battery return was started")
check(link.mission.state in ("running", "landing"),
      "mission is flying home or landing (state=%s)" % link.mission.state)

print("\n=== 4. Airborne with NO fix: lands where it is, does not just hover ===")
link = make_link()
link.mission.state = "holding"
drain(link, 3.30, height=8.0, fix=False)
check(link.mission.state == "landing",
      "no fix -> lands in place (state=%s)" % link.mission.state)

print("\n=== 5. Critical pack overrides a return in progress ===")
link = make_link()
drain(link, 3.90, height=0.2)
link.mission.state = "holding"
t = drain(link, 3.30, height=8.0, lat=-25.7500)
check(link.mission.state == "running", "returning home first")
t = drain(link, 3.00, height=8.0, lat=-25.7500, t0=t)
check(link._battery_guard.level == "land_now", "guard reaches 'land_now'")
check(link.mission.state == "landing",
      "abandons the return and lands (state=%s)" % link.mission.state)

print("\n=== 6. The pilot can always take it back ===")
link = make_link()
link.mission.state = "holding"
t = drain(link, 3.30, height=8.0, control=STICK)
check(link.mission.state == "holding",
      "while the pilot is on the sticks, nothing is commanded")
check(link._battery_override_until > 0, "a grace period was armed")

print("\n=== 7. ...and the failsafe comes back when they go passive ===")
t = t + bg.PERSIST_S + 30.0          # past BATTERY_PILOT_GRACE_S
t = drain(link, 3.30, height=8.0, control=CENTRED, t0=t)
check(link.mission.state in ("running", "landing"),
      "pilot let go -> the failsafe acts again (state=%s)" % link.mission.state)

print("\n=== 8. No barometer: warns, never commands a blind descent ===")
link = make_link()
link.mission.state = "holding"
t = time.monotonic()
end = t + 40
while t < end:
    link._latest_telemetry = {
        "battery": {"voltage_v": 3.30 * 3},
        "baro": None,
        "gps": {"has_fix": False, "lat": None, "lon": None},
    }
    link._battery_guard.update(link._latest_telemetry["battery"], now=t)
    link._battery_failsafe(CENTRED, True, now=t)
    t += 0.5
check(link._battery_guard.level in ("return", "land_now"), "guard still escalates")
check(link.mission.state == "holding",
      "but no descent is commanded without height (state=%s)" % link.mission.state)
check(link._battery_no_baro_logged is True, "and it says so, loudly, once")

print("\n=== 9. A healthy pack never touches the controls ===")
link = make_link()
link.mission.state = "holding"
drain(link, 3.85, height=8.0, seconds=60)
check(link.mission.state == "holding", "nothing commanded at 3.85 V/cell")
check(link._battery_returning is False, "no return started")

print("\n" + "-" * 55)
print("%d checks, %d failures" % (_checks, _fails))
sys.exit(1 if _fails else 0)
