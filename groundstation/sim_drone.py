"""
Simulated SeaYou drone - connects to the ground station like the real one.

Exists so the ground station, the dashboard, and the GPS/waypoint work can
all be built and tested without the aircraft: no props spinning, nothing
overheating, nothing to crash, and it works when the drone is packed away.

It speaks the exact same protocol as drone_agent.py and emits telemetry in
the exact shape unpack_telemetry() produces in server.py, so the dashboard
cannot tell the difference. If something works against this and then fails
against the real drone, the difference is real and worth chasing.

It also simulates a WORKING GPS, which the real aircraft currently does not
have (the module reports 0 satellites). That is deliberate: it means the
fly-to-coordinates feature can be written and proven now, and only needs a
working GPS module fitted to go live.

    python sim_drone.py --ground-station ws://127.0.0.1:8090/drone --token X

The physics are crude on purpose - enough to exercise the UI, the mission
logic and the failsafes, nowhere near enough to tune a PID against. Do not
use this to validate flight behaviour.
"""
import argparse
import asyncio
import json
import math
import random
import time

import aiohttp

from mission import Mission

# The battery failsafe the AIRCRAFT runs, imported rather than reimplemented.
# A simulator with its own copy of the failsafe logic tests the copy, not the
# thing that flies. It lives with the drone software; find it in either
# layout (the repository's drone/, or the working tree's "Raspberry Pi 5").
def _load_battery_guard():
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../drone", "../Raspberry Pi 5/dashboard"):
        path = os.path.normpath(os.path.join(here, rel))
        if os.path.isfile(os.path.join(path, "battery_guard.py")):
            if path not in sys.path:
                sys.path.insert(0, path)
            try:
                from battery_guard import BatteryGuard
                return BatteryGuard
            except Exception:
                return None
    return None


BatteryGuard = _load_battery_guard()

# Muizenberg beach, Cape Town - an NSRI-relevant starting point.
START_LAT = -34.1083
START_LON = 18.4700

# Rough metres per degree. Fine over the few hundred metres a drone covers.
M_PER_DEG_LAT = 111_320.0

#: Pressure the simulated aircraft sits at on the ground, hPa. The Pi
#: captures the equivalent baseline from the first packet it sees, so
#: height is always "since link-up" and never above sea level.
BARO_GROUND_HPA = round(1013.25, 1)

#: The idle state the Pi falls back to when control goes stale, matching
#: SAFE_CONTROL in server.py.
SAFE_CONTROL = {"roll": 50, "pitch": 50, "throttle": 0, "yaw": 50,
                "cmd0": 0, "cmd1": 0, "roll_trim": 25, "pitch_trim": 25}

#: Control older than this is treated as no control at all. Same value as
#: PicoLink.CONTROL_STALE_AFTER_S on the aircraft.
CONTROL_STALE_AFTER_S = 0.5

#: Quadratic drag, per metre. The horizontal model is
#:
#:     dv/dt = g * tan(tilt) - DRAG_K * v * |v|
#:
#: so the terminal speed at tilt T is sqrt(g * tan(T) / DRAG_K).
#:
#: 0.153 puts the terminal speed at the 8 degrees the guidance may command
#: at 3.0 m/s - exactly what mission.py ASSUMES when it turns a desired
#: speed into a tilt (tilt_per_ms = MAX_TILT_DEG / MAX_SPEED_MS). The
#: simulator now agrees with the guidance rather than contradicting it.
#:
#: BUT THIS IS AN ASSUMPTION, NOT A MEASUREMENT. The real aircraft's
#: tilt-to-speed relationship has never been measured, and the guidance
#: never closes the loop on actual speed - MAX_SPEED_MS only scales the
#: tilt map, so it is not a limit the aircraft is held to. Drag on a small
#: quad plausibly gives 4-8 m/s at 8 degrees, which would be FASTER than
#: the stated 3 m/s and would sail past the 2.5 m arrival radius. To see
#: what that does to a mission, set this to about 0.02 (~8 m/s) and fly
#: one on the bench.
DRAG_K = 0.153


def deg_to_nmea(value: float, is_lat: bool) -> str:
    """Decimal degrees -> the raw DDMM.MMMM / DDDMM.MMMM text the real GPS
    block carries, so the simulated packet matches the real one byte-shape."""
    hemi = ("N" if value >= 0 else "S") if is_lat else ("E" if value >= 0 else "W")
    v = abs(value)
    deg = int(v)
    minutes = (v - deg) * 60.0
    width = 2 if is_lat else 3
    return f"{deg:0{width}d}{minutes:07.4f}{hemi}"


class SimDrone:
    def __init__(self, with_gps: bool, sats: int, detect_after: float = 0.0):
        # Mission and detection both run HERE, mirroring drone_agent.py -
        # the simulator has to have the same architecture as the aircraft
        # or testing against it proves nothing about the aircraft.
        self.mission = Mission()
        # A simulated 3S pack, so the return reserve can be exercised with
        # no aircraft. Starts part-used on purpose: a full pack takes ten
        # minutes to tell you anything.
        self.pack_cell_v = 3.95
        self.guard = BatteryGuard() if BatteryGuard else None
        self._guard_returning = False
        #: Minutes of hovering the simulated pack lasts, full to empty.
        self.pack_minutes = 12.0
        #: Steady wind, as the velocity it gives the air mass (m/s, north
        #: and east). Zero unless --wind is given.
        self.wind_n = 0.0
        self.wind_e = 0.0
        self.detect_after = detect_after
        self.detection_confirmed = False
        self.mission_started_at = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = -120.0
        self.height = 0.0
        self.lat = START_LAT
        self.lon = START_LON
        # Ground velocity, metres per second, north and east. Carried
        # between steps because the aircraft now accelerates towards its
        # terminal speed instead of arriving at it instantly.
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.with_gps = with_gps
        self.sats = sats
        self.control = {"roll": 50, "pitch": 50, "throttle": 0, "yaw": 50,
                        "roll_trim": 25, "pitch_trim": 25}
        # When control last arrived. Everything about link loss hangs off
        # this, so the simulator has to track it exactly as the Pi does.
        self.control_at = 0.0
        # Mirrors --autonomous-on-link-loss on the agent: off means a lost
        # link ends the mission (by landing), on means it keeps flying.
        self.autonomous_on_link_loss = False
        # Last throttle a live pilot asked for - the only evidence, once
        # the link is stale, that the aircraft is actually in the air.
        self._last_live_throttle = 0
        self._landing_after_link_loss = False
        self.t0 = time.monotonic()
        self.motors = [0, 0, 0, 0]

    def handle_mission(self, data) -> dict:
        action = data.get("action")
        if action == "abort":
            self.mission.abort()
            return {"ok": True, "message": "aborted", **self.mission.status()}
        if action == "takeoff":
            ok, message = self.mission.takeoff(data.get("alt"), self.telemetry())
            return {"ok": ok, "message": message, **self.mission.status()}
        if action == "land":
            ok, message = self.mission.land(self.telemetry())
            return {"ok": ok, "message": message, **self.mission.status()}
        try:
            lat = float(data["lat"]); lon = float(data["lon"]); alt = float(data["alt"])
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "message": "need numeric lat, lon and alt"}
        ok, message = self.mission.start(lat, lon, alt, self.telemetry())
        if ok:
            self.mission_started_at = time.monotonic()
            self.detection_confirmed = False
        return {"ok": ok, "message": message, **self.mission.status()}

    @property
    def link_fresh(self) -> bool:
        return (time.monotonic() - self.control_at) <= CONTROL_STALE_AFTER_S

    def _select_control(self, control):
        """Mirrors GroundStationLink._select_control in drone_agent.py."""
        if self.link_fresh:
            self._last_live_throttle = int(control.get("throttle", 0) or 0)

        mission_active = self.mission.state in (
            "running", "holding", "takeoff", "landing")

        if not mission_active:
            # MANUAL FLIGHT. The case that dropped the real aircraft on
            # 2026-09-15: link died at 6.75 m, the staleness failsafe
            # handed over SAFE_CONTROL, and the motors stopped dead.
            #
            # A finished takeover lands here, and this is the ONLY path
            # that runs once the mission is idle again - so the latch is
            # cleared here or not at all.
            if self._landing_after_link_loss:
                self._landing_after_link_loss = False
                self._last_live_throttle = 0
                return control
            if (not self.link_fresh and self._last_live_throttle > 0
                    and not self.autonomous_on_link_loss):
                if self.mission.state != "landing":
                    ok, message = self.mission.land(self.telemetry())
                    print("  *** LINK LOST WHILE FLYING - %s ***"
                          % ("landing" if ok else "cannot land: %s" % message))
                    if not ok:
                        self._last_live_throttle = 0
                        return control
                    self._landing_after_link_loss = True
                guided = self.mission.step(self.telemetry(), control)
                return guided if guided is not None else control
            return control

        if self.link_fresh and _pilot_is_commanding(control):
            self.mission.abort("manual input took over")
            return control

        # Link gone: come down, do not fall. See the long comment on the
        # same branch in drone_agent.py.
        if not self.link_fresh and not self.autonomous_on_link_loss:
            if self.mission.state != "landing":
                ok, message = self.mission.land(self.telemetry())
                print("  *** LINK LOST - %s ***"
                      % ("landing" if ok else "cannot land (%s), aborting" % message))
                if not ok:
                    self.mission.abort("link lost, cannot land: %s" % message)
                    return control
            guided = self.mission.step(self.telemetry(), control)
            return guided if guided is not None else control

        # Simulated detection, so the interrupt can be exercised without a
        # camera or a person in the water.
        if (self.detect_after and not self.detection_confirmed
                and self.mission_started_at
                and time.monotonic() - self.mission_started_at >= self.detect_after):
            self.detection_confirmed = True

        if self.detection_confirmed and not self.mission.interrupted:
            if self.mission.hold_here(self.telemetry(),
                                      "drowning detected - holding position"):
                print("  *** SIMULATED DROWNING DETECTION - mission interrupted ***")

        guided = self.mission.step(self.telemetry(), control)
        return guided if guided is not None else control

    def step(self, dt: float):
        # The guided output must NOT be written back into self.control.
        # self.control is what the PILOT (via the ground station) asked for;
        # feeding guidance back into it made the next cycle read its own
        # throttle-45 command as a stick input and abort the mission
        # instantly with "manual input took over". drone_agent.py works on a
        # local variable for exactly this reason - the simulator has to
        # match it or it tests something the aircraft does not do.
        # Stale control is no control, exactly as the Pi's flight loop
        # treats it: SAFE_CONTROL goes in, and the autonomy hook gets the
        # last word on what actually goes out.
        c = self._select_control(
            self.control if self.link_fresh else dict(SAFE_CONTROL))
        thr = float(c.get("throttle", 0))

        # Commanded angles: the wire uses 0-100 with 50 centre, and the
        # firmware treats full deflection as roughly 30 degrees.
        cmd_roll = (float(c.get("roll", 50)) - 50) / 50.0 * 30.0
        cmd_pitch = (float(c.get("pitch", 50)) - 50) / 50.0 * 30.0
        cmd_yaw_rate = (float(c.get("yaw", 50)) - 50) / 50.0 * 90.0

        # First-order lag towards the commanded attitude. Real attitude
        # response is a closed loop with overshoot; this is only shaped
        # like it, which is all the UI needs.
        k = min(1.0, dt * 6.0)
        airborne = self.height > 0.05
        self.roll += (cmd_roll - self.roll) * k if airborne else -self.roll * k
        self.pitch += (cmd_pitch - self.pitch) * k if airborne else -self.pitch * k
        if airborne:
            self.yaw = (self.yaw + cmd_yaw_rate * dt + 180) % 360 - 180

        # Vertical: about 45% throttle hovers, above that it climbs.
        #
        # Only the VERTICAL part of the thrust holds the aircraft up, so
        # leaning over to travel costs lift: thrust * cos(roll) * cos(pitch).
        # This used to be `(thr - HOVER) * 0.06` with no tilt in it at all,
        # which meant the simulator could never show the aircraft sinking
        # as it leaned into a leg - so it could never show whether height
        # hold survived one either.
        HOVER = 45.0
        lift = thr * math.cos(math.radians(self.roll)) * math.cos(math.radians(self.pitch))
        climb = (lift - HOVER) * 0.06 if thr > 1 else -2.0
        self.height = max(0.0, self.height + climb * dt)

        # Horizontal: tilt ACCELERATES the aircraft, drag limits it, and
        # the velocity is integrated into the GPS position.
        #
        # This used to read
        #     speed_fwd = tan(pitch) * 9.81 * 0.6
        # which takes g*tan(tilt) - an acceleration in m/s^2 - and uses it
        # directly as a velocity in m/s. At the 8 degrees the guidance
        # commands that settled at 0.86 m/s, against the 3 m/s the guidance
        # assumes: 3.5x too slow. Every mission longer than about 103 m then
        # hit MISSION_TIMEOUT_S and aborted just short of its waypoint, which
        # looked like a bug in the mission limits and was not one. The
        # aircraft also reached its speed instantly, so nothing here ever
        # exercised acceleration or overshoot.
        #
        # Pitch moves along the current heading; roll moves across it. The
        # direction convention is unchanged - only the physics is.
        if airborne:
            acc_fwd = math.tan(math.radians(self.pitch)) * 9.81
            acc_rgt = math.tan(math.radians(self.roll)) * 9.81
            hdg = math.radians(self.yaw)
            acc_n = acc_fwd * math.cos(hdg) - acc_rgt * math.sin(hdg)
            acc_e = acc_fwd * math.sin(hdg) + acc_rgt * math.cos(hdg)

            # Drag opposes the velocity VECTOR and grows with its square, so
            # each component is damped by DRAG_K * v_component * |v|.
            speed = math.hypot(self.vel_n, self.vel_e)
            self.vel_n += (acc_n - DRAG_K * self.vel_n * speed) * dt
            self.vel_e += (acc_e - DRAG_K * self.vel_e * speed) * dt

            # A multirotor moves WITH the air it sits in: ground velocity is
            # its own air velocity plus the wind. Without this there was
            # nothing for position hold to fight, and a hold that has never
            # been pushed proves nothing.
            ground_n = self.vel_n + self.wind_n
            ground_e = self.vel_e + self.wind_e
            self.lat += (ground_n * dt) / M_PER_DEG_LAT
            self.lon += (ground_e * dt) / (M_PER_DEG_LAT * math.cos(math.radians(self.lat)))
        else:
            # On the ground. Not coasting through the landing.
            self.vel_n = self.vel_e = 0.0

        base = int(thr * 10)
        spread = int(abs(self.roll) + abs(self.pitch))
        self.motors = [max(0, min(1000, base + random.randint(-spread, spread)))
                       for _ in range(4)] if thr > 0 else [0, 0, 0, 0]

        self._step_battery(dt, thr)


    def _step_battery(self, dt: float, throttle: float):
        """Drain the simulated pack, then let the REAL guard decide.

        The drain is deliberately crude - hovering costs a steady rate and
        throttle above hover costs more. It is not a discharge model; it
        exists so the return reserve has something realistic to measure,
        and so a search can be flown until the aircraft decides it has to
        come home.
        """
        # Home capture, mirroring drone_agent.py: wherever it sits on the
        # ground with a fix is home. Without this the guard has no distance
        # to reserve against and silently falls back to the fixed threshold,
        # which is exactly the behaviour the reserve exists to replace.
        if self.with_gps and self.height < 0.3:
            self.mission.set_home(self.lat, self.lon)

        if self.height <= 0.05 and throttle <= 0:
            return
        # Full to empty over pack_minutes of hovering, worse under power.
        span_v = 4.15 - 3.10
        per_s = span_v / (self.pack_minutes * 60.0)
        load = 0.6 + (throttle / 45.0)
        self.pack_cell_v = max(2.8, self.pack_cell_v - per_s * load * dt)

        if self.guard is None:
            return

        # Sag under load, so the guard sees the same awkward signal the
        # real one does rather than a clean ramp.
        sag = 0.12 * (throttle / 50.0)
        measured = max(2.5, self.pack_cell_v - sag)
        battery = {"voltage_v": round(measured * 3, 3)}

        distance_home_m = None
        if self.with_gps and self.mission.home is not None:
            north = (self.lat - self.mission.home[0]) * M_PER_DEG_LAT
            east = ((self.lon - self.mission.home[1]) * M_PER_DEG_LAT
                    * math.cos(math.radians(self.lat)))
            distance_home_m = math.hypot(north, east)

        level = self.guard.update(battery, distance_home_m=distance_home_m)

        # Act on it exactly as drone_agent.py does: land_now beats a
        # return in progress, and a return cancels whatever the mission
        # was doing - including a search someone is part way through.
        if self.height < 0.3:
            return
        if level == "land_now":
            if self.mission.state != "landing":
                self.mission.land(self.telemetry())
        elif level == "return" and not self._guard_returning:
            ok, _ = self.mission.return_home(self.telemetry())
            self._guard_returning = ok

    def telemetry(self) -> dict:
        # Sea-level pressure falling with height, matching the real
        # barometer's ~0.1 hPa resolution so the UI sees the same
        # roughly-1-metre stepping it will see in the field.
        hpa = round(1013.25 * (1 - self.height / 44330.0) ** 5.255, 1)
        # Height derived FROM THAT QUANTISED PRESSURE, exactly as
        # _apply_relative_height() on the Pi derives it - not from
        # self.height, which is the simulator's own exact value and is
        # about a hundred times finer than the aircraft can report.
        #
        # The chain on the real drone is:
        #   firmware  press_scaled = (uint16)(pascals / 10)   -> 10 Pa steps
        #   Pi        pressure_hpa = press_scaled / 10
        #   Pi        height_m     = 44330 * (1 - (p/p0) ** (1/5.255))
        # and 10 Pa is 0.83 m of height. Anything that holds or measures
        # height has to work with that, so the simulator reports it.
        baro_height = 44330.0 * (1.0 - (hpa / BARO_GROUND_HPA) ** (1.0 / 5.255))
        noise = lambda s: random.uniform(-s, s)
        return {
            "attitude": {
                "roll": round(self.roll + noise(0.15), 2),
                "pitch": round(self.pitch + noise(0.15), 2),
                "yaw": round(self.yaw + noise(0.2), 2),
            },
            "gps": {
                "fix": "3" if self.with_gps else None,
                "has_fix": self.with_gps,
                "sat_count": self.sats if self.with_gps else 0,
                "lat_raw": deg_to_nmea(self.lat, True) if self.with_gps else None,
                "lon_raw": deg_to_nmea(self.lon, False) if self.with_gps else None,
                # Decimal degrees. The real server.py emits these too, now
                # from gps.py and the u-blox USB dongle rather than from
                # the Pico's (faulty) GPS block. --no-gps still simulates
                # an aircraft that never gets a fix.
                "lat": round(self.lat, 7) if self.with_gps else None,
                "lon": round(self.lon, 7) if self.with_gps else None,
                # The rest of what gps.py publishes, so the map and the
                # GPS panel can be built and demoed against the sim.
                "source": "pi_usb",
                "alt_m": round(baro_height + 1400.0, 1) if self.with_gps else None,
                "sats_in_view": self.sats + 3 if self.with_gps else 2,
                "hdop": 1.1 if self.with_gps else None,
                "speed_ms": (round(math.hypot(self.vel_n, self.vel_e), 2)
                             if self.with_gps else None),
                "course_deg": (round(math.degrees(
                    math.atan2(self.vel_e, self.vel_n)) % 360.0, 1)
                    if self.with_gps and math.hypot(self.vel_n, self.vel_e) >= 0.5
                    else None),
                "ts": time.time(),
            },
            "baro": {"temp_c": 22, "pressure_hpa": hpa,
                     "height_m": round(baro_height, 2)},
            "level": {"state": "idle", "busy": False, "result": None,
                      "roll_offset_deg": -4.5, "pitch_offset_deg": 0.12},
            "battery": {"voltage_v": round(self.pack_cell_v * 3, 3),
                        "cells": 3,
                        "cell_v": round(self.pack_cell_v, 3),
                        "note": "simulated pack"},
            "battery_guard": self.guard.status() if self.guard else None,
            "i2c_faults": 0,
            "gyro_cal_ok": True,
            "ekf_nan": False,
            "accel_raw": [0, 0, 4096],
            "accel_peak": [200, 200, 4400],
            "mag_ut": 28.4,
            "motor_out_debug": self.motors,
            "rx_echo_debug": [36, self.control.get("roll", 50), self.control.get("pitch", 50),
                              self.control.get("throttle", 0), self.control.get("yaw", 50),
                              0, 0, 0, 0, 0, 25, 25, 42],
            "ts": time.time(),
            "simulated": True,
            "mission": self.mission.status(),
            "detection": {
                "enabled": bool(self.detect_after),
                "ready": bool(self.detect_after),
                "error": None,
                "confirmed": self.detection_confirmed,
                "frames": 0, "avg_ms": None, "fps": None, "last": None,
                "alert": 0.9 if self.detection_confirmed else 0.0,
            },
        }


def _pilot_is_commanding(ctrl) -> bool:
    return (
        int(ctrl.get("throttle", 0)) > 0
        or abs(int(ctrl.get("roll", 50)) - 50) > 2
        or abs(int(ctrl.get("pitch", 50)) - 50) > 2
        or abs(int(ctrl.get("yaw", 50)) - 50) > 2
    )


async def flight_loop(sim):
    """The aircraft flies whether or not anything is listening.

    On the Pi this is PicoLink.run(), a 30 Hz loop that keeps talking to
    the Pico regardless of the network. Running it here as its own task,
    rather than inside the telemetry sender, is what lets the simulator
    show what a lost link actually does.
    """
    period = 1.0 / 30
    last = time.monotonic()
    last_report = 0.0
    while True:
        now = time.monotonic()
        sim.step(min(now - last, 0.1))
        last = now
        # With the link down nothing else can see the aircraft, so say what
        # it is doing. This is the only window into the exact situation
        # that matters most.
        if (not sim.link_fresh and sim.mission.state != "idle"
                and now - last_report >= 1.0):
            last_report = now
            print("  [no link] state=%-8s height=%.2f m  motors=%d"
                  % (sim.mission.state, sim.height,
                     sum(sim.motors) // len(sim.motors)))
        await asyncio.sleep(period)


async def run(args):
    sim = SimDrone(not args.no_gps, args.sats, args.detect_after)
    sim.pack_minutes = max(0.5, args.pack_minutes)
    if args.wind:
        # Meteorological convention: the direction the wind blows FROM.
        # A westerly (--wind-from 270) pushes the aircraft east.
        to = math.radians((args.wind_from + 180.0) % 360.0)
        sim.wind_n = args.wind * math.cos(to)
        sim.wind_e = args.wind * math.sin(to)
    sim.autonomous_on_link_loss = args.autonomous_on_link_loss
    asyncio.ensure_future(flight_loop(sim))
    url = args.ground_station
    if args.token:
        url += ("&" if "?" in url else "?") + "token=" + args.token

    backoff = 1.0
    while True:
        try:
            async with aiohttp.ClientSession() as session:
                print(f"Connecting to {args.ground_station} ...")
                async with session.ws_connect(url, heartbeat=5) as ws:
                    print("Connected. Simulating"
                          + (" WITH GPS fix." if sim.with_gps else " with NO GPS (like the real drone today)."))
                    backoff = 1.0

                    async def rx():
                        async for msg in ws:
                            if msg.type != aiohttp.WSMsgType.TEXT:
                                continue
                            try:
                                p = json.loads(msg.data)
                            except json.JSONDecodeError:
                                continue
                            if p.get("type") == "control":
                                sim.control = p.get("data") or sim.control
                                sim.control_at = time.monotonic()
                            elif p.get("type") == "mission":
                                await ws.send_str(json.dumps({
                                    "type": "mission_ack",
                                    "data": sim.handle_mission(p.get("data") or {})}))

                    async def tx():
                        # Telemetry only. The physics are NOT stepped here -
                        # see flight_loop(). Stepping inside the sender meant
                        # a dropped socket froze the aircraft in mid-air,
                        # which the real one does not do and which made link
                        # loss impossible to test.
                        while True:
                            await ws.send_str(json.dumps(
                                {"type": "telemetry", "data": sim.telemetry()}))
                            await asyncio.sleep(1.0 / 30)

                    await asyncio.gather(rx(), tx())
        except asyncio.CancelledError:
            raise
        except Exception as e:
            print(f"Link failed ({type(e).__name__}: {e}); retrying in {backoff:.0f}s")
        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, 15.0)


def main():
    ap = argparse.ArgumentParser(description="Simulated SeaYou drone")
    ap.add_argument("--ground-station", default="ws://127.0.0.1:8090/drone")
    ap.add_argument("--token", default="")
    ap.add_argument("--no-gps", action="store_true",
                    help="simulate the real aircraft's dead GPS (0 satellites)")
    ap.add_argument("--sats", type=int, default=11)
    ap.add_argument("--wind", type=float, default=0.0,
                    help="steady wind speed, m/s. Something for position "
                         "hold to fight; 0 (the default) is flat calm.")
    ap.add_argument("--wind-from", type=float, default=270.0,
                    help="direction the wind blows FROM, degrees true "
                         "(270 = westerly, pushing the aircraft east)")
    ap.add_argument("--pack-minutes", type=float, default=12.0,
                    help="how long the simulated pack lasts hovering, "
                         "minutes. Set it low (2-3) to watch the return "
                         "reserve cancel a search without waiting.")
    ap.add_argument("--autonomous-on-link-loss", action="store_true",
                    help="keep flying a mission when the ground station "
                         "goes away, matching the agent's flag of the same "
                         "name. Off by default: a lost link lands.")
    ap.add_argument("--detect-after", type=float, default=0.0,
                    help="seconds into a mission at which to fake a confirmed "
                         "drowning detection, so the interrupt can be tested "
                         "without a camera. 0 disables.")
    args = ap.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
