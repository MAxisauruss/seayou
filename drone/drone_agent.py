"""
SeaYou drone agent - runs ON THE PI. Flies the aircraft; talks to a ground
station when one is reachable.

The counterpart to groundstation/server.py. Three jobs, in order of how
much they matter:

  1. Exchange control and telemetry with the Pico (inherited from PicoLink).
  2. Run waypoint guidance ONBOARD.
  3. Run drowning detection ONBOARD, and interrupt the waypoint plan to
     hover over anyone it finds.

--------------------------------------------------------------------------
Why 2 and 3 are on the Pi and not the ground station
--------------------------------------------------------------------------
Because the rescue behaviour has to work when the link does not.

Detection on the laptop would mean a dropped connection lets the aircraft
fly straight past a drowning person. And detection onboard is pointless if
the thing it needs to interrupt - the flight plan - is being computed
somewhere else, because then the interrupt still has to round-trip through
the network. Both have to be local or neither is.

That is what the Pi 5 is carrying: everything needed to notice a person and
stop, with the ground station reduced to a display and a command input.

--------------------------------------------------------------------------
Why it subclasses PicoLink rather than reimplementing
--------------------------------------------------------------------------
Everything that actually keeps the drone in the air already lives in
server.py and has been flown - the 30 Hz serial loop, the compass
calibration handshake, the write-timeout handling for a hung Pico, the
0.5 s control-staleness failsafe, the CSV flight logging. A second copy
would drift from the first. This overrides three things - where telemetry
goes, where control comes from, and what gets sent each cycle - and
inherits the rest.

Safety properties, unchanged:
  * Ground station gone -> inherited staleness check sends SAFE_CONTROL
    (throttle 0) after 0.5 s. A mission does NOT keep flying on its own
    unless --autonomous-on-link-loss is given; see that flag.
  * Pico's own ~1 s no-data failsafe sits underneath that.
  * Flight logging continues with no network at all.

Run:
    python3 drone_agent.py --ground-station ws://192.168.0.162:8090/drone \\
        --token XXXX --detector-model /home/pi/best.pt
"""
import argparse
import asyncio
import json
import logging
import math
import os
import struct
import sys
import time
from pathlib import Path

import aiohttp

# Reuse, do not reimplement. See the module docstring.
from server import PicoLink, CameraStream, SAFE_CONTROL

log = logging.getLogger("drone-agent")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# mission.py lives with the ground station in the repo but is plain Python
# with no server dependencies, so it is imported from wherever it was
# deployed alongside this file.
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from mission import Mission
    from mission import ON_GROUND_M
    from battery_guard import BatteryGuard
except ImportError:
    Mission = None
    BatteryGuard = None
    ON_GROUND_M = 1.0

    log.warning("mission.py not found next to drone_agent.py - waypoint "
                "guidance disabled. Copy it from groundstation/.")


#: How long the low-pack failsafe stands back after the pilot touches the
#: sticks. The guard is latched, so this is a pause and not a cancel: if
#: they go passive again it acts. Long enough to fly to a landing spot,
#: short enough that an ignored aircraft is not left up there.
BATTERY_PILOT_GRACE_S = 15.0


class Detector:
    """Owns the detection subprocess and the decision to act on it.

    The confirmation policy lives here rather than in pi_detector.py
    because it is a flight-safety decision, not a vision one: a single
    frame is not enough to stop a mission, and a single dropped frame is
    not enough to resume it.
    """

    def __init__(self, model_path, imgsz, conf, confirm_frames, clear_frames,
                 cores=0, ramp_s=0.0):
        self.model_path = model_path
        self.imgsz = imgsz
        self.conf = conf
        self.confirm_frames = confirm_frames
        self.clear_frames = clear_frames
        # 0 = leave the scheduler alone. Otherwise the detector is pinned to
        # this many cores, optionally reached gradually - see _ramp_affinity.
        self.cores = cores
        self.ramp_s = ramp_s
        self.pinned_to = None

        self.proc = None
        self.ready = False
        self.error = None
        self.busy = False          # a frame is in flight; do not send another
        self._hits = 0
        self._misses = 0
        self.confirmed = False
        self.last = None           # most recent detection payload
        self.avg_ms = None
        self.frames = 0

    async def start(self):
        if not self.model_path:
            log.info("No --detector-model given; detection disabled.")
            return
        if not Path(self.model_path).exists():
            self.error = f"model not found: {self.model_path}"
            log.error("Detection disabled: %s", self.error)
            return
        here = Path(__file__).resolve().parent
        cmd = [sys.executable, str(here / "pi_detector.py"),
               "--model", str(self.model_path),
               "--imgsz", str(self.imgsz), "--conf", str(self.conf)]
        log.info("Starting onboard detector: %s", " ".join(cmd))
        self.proc = await asyncio.create_subprocess_exec(
            *cmd, stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE)
        asyncio.create_task(self._read_loop())
        asyncio.create_task(self._drain_stderr())
        if self.cores:
            asyncio.create_task(self._ramp_affinity())

    async def _ramp_affinity(self):
        """Widen the detector's CPU affinity gradually instead of at once.

        The load step is what a buck converter struggles with, not the
        steady-state draw: its feedback loop has finite bandwidth, so
        going from idle to four cores of inference in microseconds makes
        the rail sag before regulation catches up. Starting on one core
        and adding the rest over several seconds turns one big step into
        several small ones.

        Model load is itself a heavy burst - torch import plus weights -
        so the pin is applied immediately at spawn, before that happens,
        and only widened afterwards.

        Affinity only restricts where THIS process may run; the other
        cores stay online for the flight loop. That is deliberate: taking
        cores offline entirely would starve the thing that matters most.
        """
        try:
            total = os.cpu_count() or 4
            target = max(1, min(self.cores, total))
            start = 1 if self.ramp_s > 0 else target
            os.sched_setaffinity(self.proc.pid, set(range(start)))
            self.pinned_to = start
            log.info("Detector pinned to %d core(s)%s.", start,
                     f", widening to {target} over {self.ramp_s:.0f}s"
                     if target > start else "")
            if target <= start or self.ramp_s <= 0:
                return
            step = self.ramp_s / (target - start)
            for n in range(start + 1, target + 1):
                await asyncio.sleep(step)
                if self.proc is None or self.proc.returncode is not None:
                    return
                os.sched_setaffinity(self.proc.pid, set(range(n)))
                self.pinned_to = n
                log.info("Detector now on %d core(s).", n)
        except Exception:
            # Affinity is an optimisation. Never let it stop detection.
            log.info("Could not set detector CPU affinity - continuing unpinned.",
                     exc_info=True)

    async def _drain_stderr(self):
        """Ultralytics is chatty on stderr. Swallow it, but surface a crash."""
        try:
            while True:
                line = await self.proc.stderr.readline()
                if not line:
                    break
        except Exception:
            pass

    async def _read_loop(self):
        try:
            while True:
                line = await self.proc.stdout.readline()
                if not line:
                    break
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                kind = msg.get("type")
                if kind == "ready":
                    self.ready = True
                    log.info("Detector ready: classes=%s imgsz=%s",
                             msg.get("classes"), msg.get("imgsz"))
                elif kind == "fatal":
                    self.error = msg.get("error")
                    log.error("Detector failed: %s", self.error)
                    break
                elif kind == "frame":
                    self.busy = False
                    self.frames += 1
                    self.avg_ms = msg.get("avg_ms")
                    self.last = msg
                    self._update_confirmation(msg.get("alert", 0.0) > 0)
        except Exception:
            log.exception("detector read loop")
        finally:
            self.busy = False
            self.ready = False

    def _update_confirmation(self, hit: bool):
        """Require several consecutive frames either way.

        A single false positive must not stop a mission, and a single
        missed frame - which is common when someone is partly submerged -
        must not resume one while they are still in the water. The clear
        threshold is deliberately higher than the confirm threshold.
        """
        if hit:
            self._hits += 1
            self._misses = 0
            if self._hits >= self.confirm_frames:
                self.confirmed = True
        else:
            self._misses += 1
            self._hits = 0
            if self._misses >= self.clear_frames:
                self.confirmed = False

    async def offer(self, jpeg: bytes):
        """Hand a frame over if the detector is idle, else drop it.

        Dropping is correct: acting on a two-second-old detection is worse
        than acting on a fresh one a moment later, and a queue would grow
        without bound whenever the model is slower than the camera.
        """
        if not self.ready or self.busy or self.proc is None:
            return
        try:
            self.busy = True
            self.proc.stdin.write(struct.pack(">I", len(jpeg)) + jpeg)
            await self.proc.stdin.drain()
        except Exception:
            self.busy = False

    @property
    def status(self):
        return {
            "enabled": self.proc is not None,
            "ready": self.ready,
            "error": self.error,
            "confirmed": self.confirmed,
            "frames": self.frames,
            "avg_ms": self.avg_ms,
            "fps": round(1000.0 / self.avg_ms, 1) if self.avg_ms else None,
            "pinned_cores": self.pinned_to,
            "last": self.last.get("det") if self.last else None,
            "alert": self.last.get("alert") if self.last else 0.0,
        }

    async def stop(self):
        if self.proc and self.proc.returncode is None:
            try:
                self.proc.stdin.close()
                await asyncio.wait_for(self.proc.wait(), timeout=5)
            except Exception:
                self.proc.kill()


class GroundStationLink(PicoLink):
    """PicoLink that sends telemetry up a WebSocket, runs waypoint guidance
    onboard, and lets the detector interrupt it."""

    def __init__(self, *a, detector=None, autonomous_on_link_loss=False,
                 stabiliser=None, **kw):
        super().__init__(*a, **kw)
        self._gs = None
        self.stabiliser = stabiliser
        self.mission = Mission() if Mission else None
        self.detector = detector
        self.autonomous_on_link_loss = autonomous_on_link_loss
        self._last_interrupt_log = 0.0
        # The last throttle a LIVE pilot asked for. When the link dies this
        # is the only evidence of whether the aircraft is in the air, and
        # therefore whether cutting the motors would drop it.
        self._last_live_throttle = 0
        # True while we are flying the aircraft down after a link loss that
        # happened during MANUAL flight, so the takeover is not restarted
        # every cycle once it has finished.
        self._landing_after_link_loss = False
        # Low-pack failsafe. Runs on the Pi, needs no ground station.
        self._battery_guard = BatteryGuard() if BatteryGuard else None
        # True once a battery return has been started, so it is not restarted
        # every cycle while it flies.
        self._battery_returning = False
        # While the pilot is actively flying, the failsafe stands back. It is
        # latched, so it returns the moment they go passive again.
        self._battery_override_until = 0.0
        self._battery_no_baro_logged = False

    # --- ground station plumbing -----------------------------------
    def attach(self, ws):
        self._gs = ws

    def detach(self):
        self._gs = None
        self._latest_control = dict(SAFE_CONTROL)

    async def _broadcast(self, telemetry):
        """Overridden: one upstream socket instead of local browser sockets.
        Mission and detector state ride along so the dashboard can show what
        the aircraft decided for itself."""
        ws = self._gs
        if ws is None or ws.closed:
            return
        payload = dict(telemetry)
        if self.mission:
            payload["mission"] = self.mission.status()
        if self._battery_guard:
            payload["battery_guard"] = self._battery_guard.status()
        if self.detector:
            payload["detection"] = self.detector.status
        if self.stabiliser:
            payload["stabiliser"] = self.stabiliser.status
        try:
            await asyncio.wait_for(
                ws.send_str(json.dumps({"type": "telemetry", "data": payload})),
                timeout=1.0)
        except Exception:
            log.debug("telemetry send failed", exc_info=True)

    async def send_frame(self, jpeg: bytes):
        ws = self._gs
        if ws is None or ws.closed:
            return
        try:
            await asyncio.wait_for(ws.send_bytes(jpeg), timeout=1.0)
        except Exception:
            log.debug("frame send failed", exc_info=True)

    # --- the autonomy hook -----------------------------------------
    def _battery_failsafe(self, control, link_fresh, now=None):
        """Low-pack handling. Runs every cycle, before anything else decides.

        Entirely on the Pi: the guard reads the INA219 through telemetry the
        Pi already has, and commands the Pi's own mission code. A pack that
        goes flat with the ground station gone is still brought down.

        Returns True if it started something, so the caller knows a mission is
        now running that it did not start.

        Three things it deliberately does NOT do:

          * Act on the ground. land() commands throttle, so running it on a
            drone sitting in the grass spins the motors back up. Nothing
            happens below ON_GROUND_M.
          * Act with no barometer. Every descent this can command needs
            height. Without it the only honest thing is to warn.
          * Hold the pilot out. A stick input aborts a battery return exactly
            like any other mission - but the guard is latched, so it comes
            back after BATTERY_PILOT_GRACE_S if the pilot goes passive again.
            Authority to the pilot, without abandoning the aircraft.
        """
        if self.mission is None or self._battery_guard is None:
            return False

        telemetry = self._latest_telemetry or {}
        now = time.monotonic() if now is None else now
        level = self._battery_guard.update(telemetry.get("battery"), now=now)

        gps = telemetry.get("gps") or {}
        baro = telemetry.get("baro") or {}
        height = baro.get("height_m")

        # --- home capture ------------------------------------------------
        # Whenever the aircraft is on the ground with a fix, that is home.
        # Updated continuously rather than latched once: the last place it
        # sat on the ground is a better home than wherever it first booted.
        if (height is not None and height < ON_GROUND_M
                and gps.get("has_fix") and gps.get("lat") is not None):
            self.mission.set_home(gps["lat"], gps["lon"])

        if level not in ("return", "land_now"):
            return False

        # --- only in the air ---------------------------------------------
        if height is None:
            if not self._battery_no_baro_logged:
                log.error("BATTERY %s but there is no barometer - cannot "
                          "descend under control. Land it manually NOW.",
                          level.upper())
                self._battery_no_baro_logged = True
            return False
        if height < ON_GROUND_M:
            return False

        # --- the pilot's grace period -------------------------------------
        if now < self._battery_override_until:
            return False
        if link_fresh and _pilot_is_commanding(control):
            # Pilot is flying it. Let them, and check again shortly.
            self._battery_override_until = now + BATTERY_PILOT_GRACE_S
            return False

        state = self.mission.state

        # --- land_now beats everything, including a return in progress ----
        if level == "land_now":
            if state == "landing":
                return False
            ok, message = self.mission.land(telemetry)
            if ok:
                log.warning("BATTERY CRITICAL - landing immediately (%s)",
                            self._battery_guard.reason)
            else:
                log.error("BATTERY CRITICAL and cannot land: %s", message)
            return ok

        # --- return home --------------------------------------------------
        if self._battery_returning or state == "landing":
            return False
        ok, message = self.mission.return_home(telemetry)
        if ok:
            self._battery_returning = True
            log.warning("BATTERY LOW - %s", message)
        else:
            log.error("BATTERY LOW and cannot return or land: %s", message)
        return ok

    def _select_control(self, control: dict) -> dict:
        """Called every cycle by the inherited flight loop.

        Order of authority, highest first:
          1. The pilot. Any stick off centre or any throttle aborts the
             mission outright - no button to find, no confirmation.
          2. A confirmed drowning detection. Stops the mission travelling
             and holds position over the person.
          3. The mission, if one is running.
          4. Whatever came from the ground station.
        """
        link_fresh = (time.monotonic() - self._control_updated_at) <= self.CONTROL_STALE_AFTER_S

        # Remember what a LIVE pilot last asked for. Once the link is stale
        # `control` has already been replaced with SAFE_CONTROL by the
        # inherited failsafe, so by then it tells us nothing about whether
        # the aircraft is flying.
        if link_fresh:
            self._last_live_throttle = int(control.get("throttle", 0) or 0)

        # Low pack. Checked before the mission gate below, because it may be
        # the thing that STARTS a mission - and if it does, the guidance path
        # underneath must see it as active on this very cycle.
        self._battery_failsafe(control, link_fresh)

        # Includes takeoff and landing: both produce control, so a gate
        # that only knew about the waypoint states left them commanding
        # nothing and the aircraft sat on the ground with the state
        # cheerfully reading "takeoff".
        mission_active = bool(self.mission) and self.mission.state in (
            "running", "holding", "takeoff", "landing")

        if not mission_active:
            # ---- MANUAL FLIGHT -------------------------------------
            # A takeover that has FINISHED leaves the mission idle, which
            # lands here. Clear the latch on this path and nowhere else:
            # the mission path stops running the moment the state goes
            # idle, so a reset placed there never fires - and the aircraft
            # sat on the ground restarting the landing over and over,
            # spinning the motors back up each time. Seen on the bench.
            # This is the case that dropped the aircraft on 2026-09-15.
            # The Pi fell off the laptop hotspot at 6.75 m, the staleness
            # failsafe replaced the pilot's sticks with SAFE_CONTROL -
            # throttle 0 - and all four motors stopped dead in the air.
            #
            # Cutting the motors is the right answer for a drone sitting
            # on the ground and the wrong one for a drone that is flying.
            # The throttle the pilot last had up tells us which it is.
            if self._landing_after_link_loss:
                self._landing_after_link_loss = False
                self._last_live_throttle = 0
                return control
            if (not link_fresh
                    and self._last_live_throttle > 0
                    and self.mission is not None
                    and not self.autonomous_on_link_loss):
                return self._land_after_link_loss(control)
            return control

        # 1. Pilot override.
        if link_fresh and _pilot_is_commanding(control):
            self.mission.abort("manual input took over")
            return control

        # The link is down. Unless explicitly allowed, do not keep flying a
        # mission nobody can see or stop - but COME DOWN, do not fall.
        #
        # This used to call abort(), which means throttle 0, and the
        # inherited staleness failsafe was already holding SAFE_CONTROL
        # underneath it - so a hover at 2 m became a 2 m drop the moment
        # the WiFi hiccupped. The intent was right and the action was not:
        # landing stops the mission going anywhere AND puts the aircraft on
        # the ground under control.
        #
        # This is the whole reason guidance lives on the Pi. The flight
        # loop in server.py applies the staleness failsafe BEFORE calling
        # this hook and sends whatever this returns, so the aircraft can
        # keep flying itself with the network completely gone.
        if not link_fresh and not self.autonomous_on_link_loss:
            if self.mission.state != "landing":
                ok, message = self.mission.land(self._latest_telemetry)
                if ok:
                    log.warning("Ground station link lost - landing.")
                else:
                    # No barometer means no controlled descent is possible.
                    # Handing back with the motors idle is then genuinely
                    # the best available option, bad as it is.
                    log.error("Ground station link lost and cannot land (%s) "
                              "- aborting.", message)
                    self.mission.abort("link lost, cannot land: %s" % message)
                    return control
            # Fall straight to guidance: no detection interrupt on the way,
            # because a detection hold with no link would stop the landing
            # and leave the aircraft hovering where nobody can reach it.
            guided = self.mission.step(self._latest_telemetry, control)
            return guided if guided is not None else control

        # 2. Detection interrupt.
        if self.detector and self.detector.confirmed and not self.mission.interrupted:
            if self.mission.hold_here(self._latest_telemetry,
                                      "drowning detected - holding position"):
                log.warning("DROWNING DETECTED - mission interrupted, holding position.")

        # 3. Guidance.
        guided = self.mission.step(self._latest_telemetry, control)
        return guided if guided is not None else control

    def _land_after_link_loss(self, control):
        """Fly the aircraft down after the link died during manual flight.

        Reuses Mission's landing rather than inventing a second descent:
        controlled rate down, then a throttle ramp near the ground instead
        of a cut. Needs the barometer - with no height reference there is
        no controlled descent to fly, and handing back with the motors
        idle is then genuinely the least-bad option.
        """
        if self.mission.state != "landing":
            ok, message = self.mission.land(self._latest_telemetry)
            if not ok:
                log.error("Link lost with the throttle up, and cannot land "
                          "(%s) - falling back to the idle failsafe.", message)
                # Do not retry every cycle; the answer will not change.
                self._last_live_throttle = 0
                return control
            self._landing_after_link_loss = True
            log.warning("Ground station link lost with the throttle up - "
                        "landing the aircraft.")

        guided = self.mission.step(self._latest_telemetry, control)
        if self.mission.state == "idle":
            self._landing_after_link_loss = False
            self._last_live_throttle = 0
        return guided if guided is not None else control

    # --- commands from the ground station ---------------------------
    def handle_mission(self, payload: dict) -> dict:
        if not self.mission:
            return {"ok": False, "message": "mission.py not deployed to the Pi"}
        action = payload.get("action")
        if action == "abort":
            self.mission.abort()
            return {"ok": True, "message": "aborted", **self.mission.status()}
        if action == "takeoff":
            ok, message = self.mission.takeoff(payload.get("alt"),
                                               self._latest_telemetry)
            log.info("Takeoff %s: %s", "accepted" if ok else "REFUSED", message)
            return {"ok": ok, "message": message, **self.mission.status()}
        if action == "land":
            ok, message = self.mission.land(self._latest_telemetry)
            log.info("Land %s: %s", "accepted" if ok else "REFUSED", message)
            return {"ok": ok, "message": message, **self.mission.status()}
        try:
            lat = float(payload["lat"]); lon = float(payload["lon"]); alt = float(payload["alt"])
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "message": "need numeric lat, lon and alt"}
        ok, message = self.mission.start(lat, lon, alt, self._latest_telemetry)
        log.info("Mission start %s: %s", "accepted" if ok else "REFUSED", message)
        return {"ok": ok, "message": message, **self.mission.status()}


def _pilot_is_commanding(ctrl) -> bool:
    """True if the sticks are off centre or the throttle is up at all."""
    return (
        int(ctrl.get("throttle", 0)) > 0
        or abs(int(ctrl.get("roll", 50)) - 50) > 2
        or abs(int(ctrl.get("pitch", 50)) - 50) > 2
        or abs(int(ctrl.get("yaw", 50)) - 50) > 2
    )


class UplinkStabiliser:
    """Digital stabilisation for the frames a HUMAN watches.

    With no gimbal the camera is bolted to the airframe, so every time the
    drone tilts to move, the whole view swings. That is genuinely horrible
    to watch, and it is what this fixes.

    Two deliberate scoping decisions:

    1. **Only the uplink frames are stabilised, not the detector's.** The
       detector looks at one frame at a time and does not care where the
       horizon is; stabilising for it would burn CPU for no accuracy and
       would CROP AWAY edge pixels where a person might be. The detector
       keeps the raw frame. Camera motion is handled for the detector by
       the tracker instead (botsort.yaml compensates for it directly).

    2. **The uplink is already rate-limited** (default 10 fps, far lower on
       mobile data), so this runs on a fraction of the frames the camera
       produces. That is what makes it affordable at all.

    Method: track features between frames, estimate the rigid transform,
    smooth the trajectory, and warp back towards the smoothed path. Feature
    based rather than using the drone's own attitude, because telemetry
    quantises the quaternion to about 1.15 degrees per step - correcting
    with that would add visible stair-stepping rather than remove jerk.

    NOTE the cost: decode -> warp -> re-encode is not free. Measure before
    trusting it, and see the power warning in LIVE_FEED_QUALITY_FOR_ML.md -
    this Pi browned out under full CPU load on 2026-09-12.
    """

    def __init__(self, smoothing=0.85, crop=0.92):
        self.smoothing = smoothing
        # How much of the frame is kept. Stabilisation has to warp the image
        # within a margin, so some border is always lost.
        self.crop = crop
        self._prev_gray = None
        self._accum = None      # accumulated raw transform
        self._smooth = None     # low-passed version of it
        self.ok = False
        self.error = None
        self.avg_ms = None
        self._n = 0
        self._total = 0.0
        try:
            import cv2, numpy  # noqa: F401
            self.ok = True
        except ImportError as e:
            self.error = str(e)

    def process(self, jpeg: bytes) -> bytes:
        """Return a stabilised JPEG, or the original on any problem.

        Never raises. A stabiliser failure must degrade to unstabilised
        video, never to no video.
        """
        if not self.ok:
            return jpeg
        try:
            import cv2
            import numpy as np
            t0 = time.monotonic()

            frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                return jpeg
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if self._prev_gray is None:
                self._prev_gray = gray
                self._accum = np.zeros(3, dtype=np.float64)   # dx, dy, dangle
                self._smooth = np.zeros(3, dtype=np.float64)
                return jpeg

            prev_pts = cv2.goodFeaturesToTrack(self._prev_gray, maxCorners=120,
                                               qualityLevel=0.01, minDistance=24,
                                               blockSize=7)
            transform = None
            if prev_pts is not None and len(prev_pts) >= 8:
                curr_pts, st, _ = cv2.calcOpticalFlowPyrLK(self._prev_gray, gray,
                                                           prev_pts, None)
                if curr_pts is not None:
                    good_prev = prev_pts[st == 1]
                    good_curr = curr_pts[st == 1]
                    if len(good_prev) >= 8:
                        m, _ = cv2.estimateAffinePartial2D(good_prev, good_curr)
                        if m is not None:
                            transform = np.array([m[0, 2], m[1, 2],
                                                  math.atan2(m[1, 0], m[0, 0])])
            self._prev_gray = gray
            if transform is None:
                # Featureless scene - open water is exactly this. Leave the
                # frame alone rather than warping on a bad estimate.
                return jpeg

            self._accum += transform
            # Low-pass the trajectory. The difference between where the
            # camera actually is and where a smooth path would put it is the
            # correction to apply.
            self._smooth = self.smoothing * self._smooth + (1 - self.smoothing) * self._accum
            diff = self._smooth - self._accum

            da = diff[2]
            h, w = frame.shape[:2]
            m = np.array([
                [math.cos(da), -math.sin(da), diff[0]],
                [math.sin(da),  math.cos(da), diff[1]],
            ], dtype=np.float64)
            out = cv2.warpAffine(frame, m, (w, h), borderMode=cv2.BORDER_REPLICATE)

            # Crop and rescale so the warped-in border never shows.
            cw, ch = int(w * self.crop), int(h * self.crop)
            x0, y0 = (w - cw) // 2, (h - ch) // 2
            out = cv2.resize(out[y0:y0 + ch, x0:x0 + cw], (w, h))

            ok, enc = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                return jpeg
            self._n += 1
            self._total += (time.monotonic() - t0) * 1000.0
            self.avg_ms = round(self._total / self._n, 1)
            return enc.tobytes()
        except Exception:
            log.debug("stabiliser", exc_info=True)
            return jpeg

    @property
    def status(self):
        return {"enabled": self.ok, "error": self.error,
                "avg_ms": self.avg_ms, "frames": self._n}


class VideoRecorder:
    """Writes the camera stream to a file on the Pi.

    Video was previously only streamed and never kept, so nothing survived a
    flight. That matters for three separate reasons: reviewing a crash,
    footage for the report, and - the reason this was added now - because
    Gyroflow is a POST-processing tool and cannot stabilise anything unless
    there is a recorded file to stabilise.

    Format is a raw MJPEG stream: the frames already arrive as JPEGs from
    picamera2, so they are written straight through with no re-encoding, no
    CPU cost, and no extra dependency on the Pi. Convert afterwards with:

        ffmpeg -f mjpeg -r 30 -i flight_XXXX.mjpeg -c:v libx264 out.mp4

    A timestamp index is written beside it, because a raw MJPEG stream
    carries no timing of its own - without it the footage cannot be lined
    up against the flight log, which is exactly what any gyro-based
    stabilisation needs.
    """

    def __init__(self, directory, max_mb):
        self.dir = Path(directory)
        self.max_bytes = max_mb * 1024 * 1024
        self._f = None
        self._index = None
        self.path = None
        self.frames = 0
        self.bytes = 0
        self._failed = False

    def _open(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = self.dir / f"flight_{stamp}.mjpeg"
        self._f = open(self.path, "wb")
        self._index = open(self.dir / f"flight_{stamp}.frames.csv", "w", buffering=1)
        self._index.write("frame,wall_clock_epoch,byte_offset,length\n")
        log.info("Recording video: %s", self.path)

    def write(self, jpeg: bytes):
        if self._failed:
            return
        try:
            if self._f is None:
                self._open()
            # Stop rather than fill the SD card - a full card would take the
            # flight log down with it, and the log matters more than footage.
            if self.bytes + len(jpeg) > self.max_bytes:
                log.warning("Video recording hit its %d MB limit - stopping.",
                            self.max_bytes // (1024 * 1024))
                self.close()
                self._failed = True
                return
            self._index.write(f"{self.frames},{time.time():.3f},{self.bytes},{len(jpeg)}\n")
            self._f.write(jpeg)
            self.bytes += len(jpeg)
            self.frames += 1
        except Exception:
            # Never let a recording problem reach the flight loop.
            self._failed = True
            log.exception("Video recording failed - continuing without it")

    def close(self):
        for h in (self._f, self._index):
            try:
                if h:
                    h.close()
            except Exception:
                pass
        self._f = self._index = None

    @property
    def status(self):
        return {"recording": self._f is not None, "frames": self.frames,
                "mb": round(self.bytes / 1048576, 1),
                "path": str(self.path) if self.path else None}


async def camera_pump(link, camera, detector, uplink_fps, recorder=None,
                      stabiliser=None):
    """One camera reader feeding two consumers.

    The camera can only be opened once, so frames are taken here and handed
    to both the detector (onboard, every frame it can keep up with) and the
    ground station (rate-limited - on mobile data video is the expensive
    part, and control must never queue behind it).
    """
    if not camera.available:
        log.info("No camera - detection and video are both unavailable.")
        return
    period = 1.0 / max(uplink_fps, 0.1)
    last_uplink = 0.0
    log.info("Camera pump running (uplink ~%.1f fps, detector takes what it can).",
             uplink_fps)
    while True:
        try:
            frame = await camera._output.next_frame()
            # The detector gets first refusal: it is the safety-relevant
            # consumer, and it drops frames itself when busy.
            if detector:
                await detector.offer(frame)
            # Recorded at full rate - it is a straight file write of frames
            # that already exist, so it costs almost nothing, and half a
            # recording is much less useful than a whole one.
            if recorder:
                recorder.write(frame)
            now = time.monotonic()
            if now - last_uplink >= period:
                last_uplink = now
                # Stabilised only here, on the rate-limited uplink - the
                # detector above already got the raw, uncropped frame.
                await link.send_frame(
                    stabiliser.process(frame) if stabiliser else frame)
        except Exception:
            log.debug("camera pump", exc_info=True)
            await asyncio.sleep(0.5)


def _ground_stations(raw: str) -> list:
    """Split a comma-separated --ground-station into a list of addresses.

    Blank entries are dropped so a trailing comma in a systemd unit - the
    easiest typo to make in a file nobody looks at twice - cannot produce
    an empty URL that fails on every other reconnect.
    """
    urls = [u.strip() for u in (raw or "").split(",") if u.strip()]
    if not urls:
        raise SystemExit("--ground-station needs at least one address")
    return urls


async def connection_loop(link, urls, token, camera, detector, fps, recorder=None,
                          stabiliser=None):
    """Keep an outbound connection to the ground station, forever.

    `urls` is one or more addresses, tried in order and then round-robin.
    The laptop is at a different address on every network it joins - the
    house WiFi, its own hotspot - and nobody is going to edit a systemd
    unit in a field, so the aircraft tries each until one answers.
    """
    def with_token(u):
        return u if not token else "%s%stoken=%s" % (u, "&" if "?" in u else "?", token)

    backoff = 1.0
    cam_task = None
    attempt = 0

    while True:
        url = urls[attempt % len(urls)]
        full = with_token(url)
        attempt += 1
        try:
            timeout = aiohttp.ClientTimeout(total=None, sock_connect=10)
            # ThreadedResolver, explicitly. aiohttp prefers aiodns when it
            # is installed - and it IS on this Pi - which resolves through
            # c-ares, i.e. plain DNS, which cannot see mDNS ".local" names.
            # The threaded resolver goes through getaddrinfo and therefore
            # NSS, which does. That is what makes the laptop reachable by
            # name instead of by an address that changes with the network.
            connector = aiohttp.TCPConnector(resolver=aiohttp.ThreadedResolver())
            async with aiohttp.ClientSession(timeout=timeout,
                                             connector=connector) as session:
                log.info("Connecting to ground station %s ...", url)
                async with session.ws_connect(full, heartbeat=5,
                                              max_msg_size=16 * 1024 * 1024) as ws:
                    log.info("Connected to ground station.")
                    backoff = 1.0
                    link.attach(ws)
                    if cam_task is None:
                        cam_task = asyncio.create_task(
                            camera_pump(link, camera, detector, fps, recorder,
                                        stabiliser))

                    async for msg in ws:
                        if msg.type != aiohttp.WSMsgType.TEXT:
                            continue
                        try:
                            payload = json.loads(msg.data)
                        except json.JSONDecodeError:
                            continue
                        kind = payload.get("type")
                        if kind == "control":
                            link.set_control(payload.get("data", {}))
                        elif kind == "gp_debug":
                            link.set_gamepad_debug(payload.get("gp"))
                        elif kind == "level_cal":
                            link.request_level_cal(payload.get("action", "start"))
                        elif kind == "mission":
                            result = link.handle_mission(payload.get("data") or {})
                            await ws.send_str(json.dumps(
                                {"type": "mission_ack", "data": result}))
        except asyncio.CancelledError:
            raise
        except Exception as e:
            log.warning("Ground station link failed (%s: %s)", type(e).__name__, e)
        finally:
            link.detach()

        # Only back off once every address has been tried, or a single
        # unreachable entry in the list would stretch the delay before the
        # one that actually works is even attempted.
        if attempt % len(urls) == 0:
            log.info("Reconnecting in %.0fs ...", backoff)
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30.0)
        else:
            await asyncio.sleep(1.0)


async def main_async(args):
    detector = Detector(args.detector_model, args.detector_imgsz, args.detector_conf,
                        args.confirm_frames, args.clear_frames,
                        args.detector_cores, args.detector_ramp_s)
    link = GroundStationLink(args.serial, args.baud, args.compass_cal, args.log_dir,
                             detector=detector,
                             autonomous_on_link_loss=args.autonomous_on_link_loss)
    camera = CameraStream()
    camera.start()
    await detector.start()

    recorder = VideoRecorder(args.video_dir, args.video_max_mb) if args.record_video else None
    stabiliser = UplinkStabiliser(args.stabilise_smoothing) if args.stabilise_uplink else None
    if stabiliser and not stabiliser.ok:
        log.error("Uplink stabilisation unavailable: %s", stabiliser.error)
    # Assigned after construction rather than passed in, purely because the
    # stabiliser needs the parsed args and the link is built before them.
    link.stabiliser = stabiliser

    if args.autonomous_on_link_loss:
        log.warning("--autonomous-on-link-loss is ON: a mission will keep flying "
                    "with no ground station. Nobody can stop it remotely. "
                    "Only use this once the aircraft has earned it.")

    # The Pico link runs whatever the network is doing. This ordering is the
    # point of the whole design: flying does not depend on the ground
    # station being reachable.
    try:
        await asyncio.gather(
            link.run(),
            connection_loop(link, _ground_stations(args.ground_station),
                            args.token, camera, detector,
                            args.video_fps, recorder, stabiliser),
        )
    finally:
        await detector.stop()
        if recorder:
            recorder.close()


def main():
    ap = argparse.ArgumentParser(description="SeaYou drone agent (runs on the Pi)")
    ap.add_argument("--ground-station", required=True,
                    help="ws://<pc-address>:8090/drone. Several may be given, "
                         "comma-separated, and are tried in turn - useful "
                         "because the ground station's address changes with "
                         "the network it is on")
    ap.add_argument("--token", default="", help="shared secret from the ground station")
    ap.add_argument("--serial", default="/dev/pico")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--compass-cal", default="/home/pi/compass_cal.txt")
    ap.add_argument("--log-dir", default="/home/pi/flightlogs")
    ap.add_argument("--video-fps", type=float, default=10.0,
                    help="frames per second sent UPSTREAM. Lower this hard on "
                         "mobile data - control must never queue behind video. "
                         "Does not affect the onboard detector.")

    ap.add_argument("--detector-model", default=None,
                    help="path to best.pt on the Pi. Omit to disable detection.")
    ap.add_argument("--detector-imgsz", type=int, default=320,
                    help="inference size; 320 is ~4x faster than 640 on a Pi 5")
    ap.add_argument("--detector-conf", type=float, default=0.45)
    ap.add_argument("--detector-cores", type=int, default=3,
                    help="how many cores the detector may use. 0 leaves the "
                         "scheduler alone. Default 3 of 4, so the flight loop "
                         "always has one to itself.")
    ap.add_argument("--detector-ramp-s", type=float, default=8.0,
                    help="widen from 1 core to --detector-cores over this many "
                         "seconds. Turns one large current step into several "
                         "small ones, which is what a buck converter with "
                         "limited loop bandwidth can actually follow. 0 = "
                         "no ramp.")
    ap.add_argument("--confirm-frames", type=int, default=3,
                    help="consecutive detections before a mission is interrupted")
    ap.add_argument("--clear-frames", type=int, default=10,
                    help="consecutive clear frames before the alert drops. Higher "
                         "than confirm on purpose - a partly submerged person is "
                         "easy to miss for a frame or two")

    ap.add_argument("--stabilise-uplink", action="store_true",
                    help="digitally stabilise the video a HUMAN watches. With no "
                         "gimbal the camera is bolted to the frame, so every tilt "
                         "swings the view. OFF by default because it costs CPU "
                         "(decode, warp, re-encode) on a Pi that has browned out "
                         "under load - fix the power supply first. Does NOT touch "
                         "the detector, which keeps the raw uncropped frame.")
    ap.add_argument("--stabilise-smoothing", type=float, default=0.85,
                    help="0.5 barely smooths, 0.95 is very smooth but lags and "
                         "crops more")

    ap.add_argument("--record-video", action="store_true",
                    help="save the camera stream to a file on the Pi. Needed for "
                         "Gyroflow, which is post-processing only and cannot "
                         "touch a live feed")
    ap.add_argument("--video-dir", default="/home/pi/flightvideo")
    ap.add_argument("--video-max-mb", type=int, default=2000,
                    help="stop recording at this size. A full SD card would take "
                         "the flight log down with it, and the log matters more")

    ap.add_argument("--autonomous-on-link-loss", action="store_true",
                    help="keep flying a mission when the ground station is "
                         "unreachable. OFF by default: the aircraft has never "
                         "flown a mission at all, and an uncommandable drone "
                         "continuing a flight plan is how they are lost")
    args = ap.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
