"""Tests for the low-battery failsafe and return-to-home.

Runs on the Pi with no aircraft attached:

    ssh pi@drone.local 'cd /home/pi/dashboard && python3 test_battery_guard.py'

The guard can put the drone on the ground by itself, so the cases that matter
most here are the ones where it must NOT act: a punch-out sag, a glitched
reading, a missing sensor. A failsafe that fires when it should not is its own
hazard.

Time is injected rather than slept, so the whole suite runs instantly and the
persistence windows are exercised exactly.
"""

import sys

import battery_guard as bg
from battery_guard import BatteryGuard
from mission import Mission

_checks = 0
_fails = 0


def check(ok, what):
    global _checks, _fails
    _checks += 1
    if ok:
        print("  ok    %s" % what)
    else:
        _fails += 1
        print("  FAIL  %s" % what)


def pack(cell_v, cells=3, fault=None):
    """A telemetry battery dict at a given per-cell voltage."""
    if fault:
        return {"voltage_v": None, "fault": fault}
    return {"voltage_v": round(cell_v * cells, 3), "cells": cells,
            "cell_v": cell_v}


def feed(guard, cell_v, seconds, t0, step=0.5, fault=None):
    """Feed a steady voltage for `seconds`. Returns the end time."""
    t = t0
    end = t0 + seconds
    while t < end:
        guard.update(pack(cell_v, guard.cells, fault), now=t)
        t += step
    return t


# ---------------------------------------------------------------------------
print("\n=== 1. A healthy pack does nothing ===")
g = BatteryGuard()
t = feed(g, 3.90, 30, 0.0)
check(g.level == "ok", "level 'ok' at 3.90 V/cell")
check(g.cell_v is not None and abs(g.cell_v - 3.90) < 0.02,
      "filtered cell voltage tracks the input (%.3f)" % (g.cell_v or 0))

print("\n=== 2. A punch-out sag must NOT trigger anything ===")
# 15 mOhm/cell at 45 A is ~0.7 V of sag. Two seconds of it, well under the
# 3 s persistence window, then back to normal.
g = BatteryGuard()
t = feed(g, 3.85, 20, 0.0)
before = g.level
t = feed(g, 3.10, 2.0, t)          # brief hard pull, below LAND_V
t = feed(g, 3.85, 20, t)
check(before == "ok" and g.level == "ok",
      "a 2 s sag to 3.10 V/cell leaves the level at 'ok'")

print("\n=== 3. Sustained low voltage does trigger, in order ===")
g = BatteryGuard()
t = feed(g, 3.90, 20, 0.0)
check(g.level == "ok", "starts ok")
t = feed(g, 3.50, 20, t)
check(g.level == "warn", "3.50 V/cell held -> 'warn'")
t = feed(g, 3.35, 20, t)
check(g.level == "return", "3.35 V/cell held -> 'return'")
t = feed(g, 3.05, 20, t)
check(g.level == "land_now", "3.05 V/cell held -> 'land_now'")

print("\n=== 4. Response time: slow to react to a step, not slow to act ===")
# A step from 3.90 to 3.30 V/cell is not a discharge, it is a load transient,
# and the filter is meant to lag it. The combined lag is the filter reaching
# the threshold plus PERSIST_S. Measured rather than assumed, because it is
# the number that says how fast this failsafe actually is.
g = BatteryGuard()
t = feed(g, 3.90, 30, 0.0)
step_at = t
t = feed(g, 3.30, bg.PERSIST_S - 1.0, t)
check(g.level == "ok", "2 s below RETURN_V is still 'ok' - transients rejected")

fired_at = None
while t < step_at + 60 and fired_at is None:
    g.update(pack(3.30, g.cells), now=t)
    if g.level == "return":
        fired_at = t
    t += 0.5
check(fired_at is not None, "a sustained 3.30 V/cell does eventually trigger 'return'")
if fired_at:
    print("        step response: %.1f s from the step to the return"
          % (fired_at - step_at))
    check(fired_at - step_at < 20.0, "  and it acts within 20 s of the step")

print("\n=== 4b. A REAL discharge, which is what actually happens ===")
# A pack does not step - it declines. The filter tracks a slow fall closely,
# so the lag is small. This is the case that matters in flight.
g = BatteryGuard()
t = feed(g, 3.70, 30, 0.0)
v = 3.70
crossed_at = None
fired_at = None
while v > 3.00 and fired_at is None:
    v -= 0.005
    g.update(pack(round(v, 4), g.cells), now=t)
    if crossed_at is None and v <= bg.RETURN_V:
        crossed_at = t
    if g.level == "return":
        fired_at = t
    t += 0.5
check(fired_at is not None, "a real decline triggers 'return'")
if fired_at and crossed_at:
    lag = fired_at - crossed_at
    print("        decline response: %.1f s after the pack truly crossed %.2f V/cell"
          % (lag, bg.RETURN_V))
    check(lag < 15.0, "  lag on a real discharge is under 15 s")

print("\n=== 5. The latch: voltage recovery must NOT cancel a return ===")
# This is the oscillation case. Trigger a return, then let the voltage climb
# back as the throttle comes off during the descent.
g = BatteryGuard()
t = feed(g, 3.90, 20, 0.0)
t = feed(g, 3.30, 20, t)
check(g.level == "return", "return triggered")
t = feed(g, 3.95, 60, t)           # long recovery, well above every threshold
check(g.level == "return", "voltage back to 3.95 V/cell does NOT clear 'return'")
check(g.cell_v > 3.8, "  (the filtered voltage did recover: %.2f)" % g.cell_v)

print("\n=== 6. Escalation still works while latched ===")
t = feed(g, 3.00, 20, t)
check(g.level == "land_now", "can still escalate 'return' -> 'land_now'")
t = feed(g, 3.95, 40, t)
check(g.level == "land_now", "and 'land_now' never goes back either")

print("\n=== 7. reset() clears it, for a pack change on the ground ===")
g.reset()
check(g.level == "unknown", "reset -> 'unknown'")
t = feed(g, 3.90, 20, t)
check(g.level == "ok", "picks up again from a fresh pack")

print("\n=== 8. Nonsense readings are rejected, not acted on ===")
g = BatteryGuard()
t = feed(g, 3.90, 20, 0.0)
lvl = g.level
# 4 V total on a 3S is 1.33 V/cell - impossible; a wrong PACK_CELLS or a
# glitched read. Must NOT be treated as a flat pack.
for _ in range(40):
    g.update({"voltage_v": 4.0}, now=t)
    t += 0.5
check(g.level == lvl, "1.33 V/cell is ignored, level unchanged (%s)" % g.level)
for _ in range(40):
    g.update({"voltage_v": 99.0}, now=t)
    t += 0.5
check(g.level == lvl, "33 V/cell is ignored too")

print("\n=== 9. A missing or faulted sensor holds, and never invents a level ===")
g = BatteryGuard()
t = feed(g, 3.90, 20, 0.0)
check(g.level == "ok", "ok before the sensor drops out")
t = feed(g, 0, 40, t, fault="i2c_error")
check(g.level == "ok", "an I2C fault does NOT trigger a landing")
g2 = BatteryGuard()
for _ in range(40):
    g2.update(None, now=t)
    t += 0.5
check(g2.level == "unknown", "no sensor at all stays 'unknown', never 'land_now'")

print("\n=== 10. Thresholds sit where the data sheet allows ===")
check(bg.LAND_V > bg.CELL_DATASHEET_CUTOFF_V,
      "land threshold %.2f V is above the %.2f V data-sheet cut-off"
      % (bg.LAND_V, bg.CELL_DATASHEET_CUTOFF_V))
check(bg.WARN_V > bg.RETURN_V > bg.LAND_V,
      "warn > return > land_now, each with room to act")
check(bg.LAND_V - bg.CELL_DATASHEET_CUTOFF_V >= 0.5,
      "at least 0.5 V/cell of reserve below the land threshold")

# ---------------------------------------------------------------------------
print("\n=== 11. return_home falls back to landing when it cannot navigate ===")
FIX = {"gps": {"has_fix": True, "lat": -25.7485, "lon": 28.1870},
       "baro": {"height_m": 5.0, "pressure_hpa": 870.0, "temp_c": 25}}
NOFIX = {"gps": {"has_fix": False, "lat": None, "lon": None},
         "baro": {"height_m": 5.0, "pressure_hpa": 870.0, "temp_c": 25}}

m = Mission()
m.state = "holding"
ok, msg = m.return_home(NOFIX)
check(ok and m.state == "landing",
      "no home recorded -> lands where it is (%s)" % msg)

m = Mission()
m.set_home(-25.7485, 28.1870)
m.state = "holding"
ok, msg = m.return_home(NOFIX)
check(ok and m.state == "landing",
      "home known but no fix -> lands where it is (%s)" % msg)

print("\n=== 12. return_home navigates when it can ===")
m = Mission()
m.set_home(-25.7500, 28.1870)      # ~165 m south of the current position
m.state = "holding"
ok, msg = m.return_home(FIX)
check(ok and m.state == "running", "with home and a fix -> flies home (%s)" % msg)
check(m.target == (-25.7500, 28.1870), "target is home")
check(m._land_on_arrival is True, "armed to land on arrival")

print("\n=== 13. Already at home -> just land ===")
m = Mission()
m.set_home(-25.7485, 28.1870)
m.state = "holding"
ok, msg = m.return_home(FIX)
check(ok and m.state == "landing", "within the arrive radius -> lands (%s)" % msg)

print("\n=== 14. Home beyond the range limit -> land here, do not set off ===")
m = Mission()
m.set_home(-28.0, 28.1870)         # ~250 km away
m.state = "holding"
ok, msg = m.return_home(FIX)
check(m.state == "landing", "refuses an out-of-range return and lands (%s)" % msg)

print("\n" + "-" * 55)
print("%d checks, %d failures" % (_checks, _fails))
sys.exit(1 if _fails else 0)
