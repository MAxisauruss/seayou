"""
SeaYou ground station - runs on the PC, hosts the dashboard.

This inverts the original architecture. Before, the Pi hosted the web
server and browsers connected IN to it. Here the PC hosts, and the drone
dials OUT to the PC.

That inversion is the whole point, for two reasons:

  1. The drone never needs an inbound address. On a home LAN that is a
     convenience; on mobile data it is a hard requirement, because
     carriers put SIMs behind CGNAT where nothing can reach them from
     outside. A drone that dials out works anywhere it has any internet
     connection at all.
  2. The dashboard lives on a machine you control and can hand to other
     people, rather than on the aircraft.

What did NOT move: the Pico serial link, the 30 Hz control loop, the
failsafes and the flight logging all stay on the Pi (see drone_agent.py).
Those need hard timing and must keep working when the network does not -
if this ground station disappears mid-flight, the Pi falls back to
SAFE_CONTROL on its own after 0.5 s, exactly as it always did.

Run:
    python server.py [--port 8080] [--webapp ../seayou-main/dist]
"""
import argparse
import asyncio
import json
import logging
import secrets
import sys
import time
from pathlib import Path

from aiohttp import web, WSMsgType

from auth import Users
from login_page import LOGIN_HTML

log = logging.getLogger("groundstation")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# Same safe idle state the Pi uses. Sent to the drone whenever no browser
# is actively flying, and it is what the drone falls back to by itself if
# this server goes away.
SAFE_CONTROL = {
    "roll": 50, "pitch": 50, "throttle": 0, "yaw": 50,
    "cmd0": 0, "cmd1": 0, "roll_trim": 25, "pitch_trim": 25,
}

# A browser that stops sending control is treated as gone this quickly.
CONTROL_STALE_AFTER_S = 0.5

# Control packets per second pushed to the drone. Matches the rate the
# dashboard and the Pi have always used.
CONTROL_HZ = 30


class Hub:
    """Relays between the one connected drone and any number of browsers.

    Deliberately dumb: it forwards, it does not interpret. The flight
    logic stays on the Pi and in the browser, both of which already exist
    and are tested. Anything clever added here would be a third place
    that has to agree with the other two.
    """

    def __init__(self, token=None):
        self.token = token
        self._drone = None
        self._browsers = set()
        self._latest_telemetry = None
        self._latest_frame = None
        self._frame_event = asyncio.Event()
        self._latest_control = dict(SAFE_CONTROL)
        self._control_at = 0.0
        self._frames = 0
        # Mission state as last REPORTED by the drone. The ground station
        # no longer computes it - see control_pump.
        self._mission = {"state": "idle", "reason": "", "target": None,
                         "distance_m": None, "bearing_deg": None, "elapsed_s": 0}
        self._detection = {"enabled": False}
        # Resolved when the drone acknowledges a mission command.
        self._mission_ack = None
        # What was last actually sent to the drone. Distinct from
        # _latest_control, which is only what the PILOT asked for - during a
        # mission the two differ, and reporting the pilot's input as though
        # it were the command in force is actively misleading.
        self._sent_control = dict(SAFE_CONTROL)

    # --- drone side -------------------------------------------------
    @property
    def drone_connected(self):
        return self._drone is not None

    async def attach_drone(self, ws):
        if self._drone is not None:
            # One aircraft. A second connection is almost certainly a stale
            # session that has not timed out yet - drop the old one rather
            # than have two sockets both think they own the link.
            log.warning("A second drone connected; dropping the previous one.")
            try:
                await self._drone.close()
            except Exception:
                pass
        self._drone = ws
        log.info("Drone connected.")
        await self._tell_browsers({"type": "drone_status", "data": {"connected": True}})

    async def detach_drone(self, ws):
        if self._drone is ws:
            self._drone = None
            self._latest_telemetry = None
            self._latest_frame = None
            log.warning("Drone disconnected.")
            await self._tell_browsers({"type": "drone_status", "data": {"connected": False}})

    async def on_drone_telemetry(self, data):
        self._latest_telemetry = data
        if isinstance(data.get("mission"), dict):
            self._mission = data["mission"]
        if isinstance(data.get("detection"), dict):
            self._detection = data["detection"]
        await self._tell_browsers({"type": "telemetry", "data": data})

    def on_drone_frame(self, jpeg):
        self._latest_frame = jpeg
        self._frames += 1
        self._frame_event.set()
        self._frame_event.clear()

    # --- browser side -----------------------------------------------
    def add_browser(self, ws):
        self._browsers.add(ws)

    def remove_browser(self, ws):
        self._browsers.discard(ws)
        if not self._browsers:
            # Nobody is flying; stop commanding immediately rather than
            # waiting for the staleness timeout.
            self._latest_control = dict(SAFE_CONTROL)

    def set_control(self, control):
        merged = dict(SAFE_CONTROL)
        merged.update(control or {})
        self._latest_control = merged
        self._control_at = time.monotonic()

    async def send_to_drone(self, msg):
        ws = self._drone
        if ws is None:
            return False
        try:
            await asyncio.wait_for(ws.send_str(json.dumps(msg)), timeout=1.0)
            return True
        except Exception:
            log.exception("Failed sending to the drone")
            return False

    async def send_mission(self, data) -> dict:
        """Send a mission command to the drone and wait for its answer.

        The drone is the authority: it holds the GPS fix, the guidance and
        the limits, so it decides whether a mission is acceptable. Waiting
        for its reply means the operator sees the real refusal reason
        rather than an optimistic 'sent'.
        """
        if not self.drone_connected:
            return {"ok": False, "message": "drone not connected"}
        loop = asyncio.get_running_loop()
        self._mission_ack = loop.create_future()
        if not await self.send_to_drone({"type": "mission", "data": data}):
            self._mission_ack = None
            return {"ok": False, "message": "could not reach the drone"}
        try:
            return await asyncio.wait_for(self._mission_ack, timeout=3.0)
        except asyncio.TimeoutError:
            return {"ok": False, "message": "drone did not answer"}
        finally:
            self._mission_ack = None

    def on_mission_ack(self, data):
        if self._mission_ack and not self._mission_ack.done():
            self._mission_ack.set_result(data)

    async def _tell_browsers(self, msg):
        if not self._browsers:
            return
        payload = json.dumps(msg)
        dead = []
        for ws in self._browsers:
            try:
                # Bounded: a laptop that sleeps or leaves WiFi leaves a
                # half-open socket that would otherwise stall this loop.
                await asyncio.wait_for(ws.send_str(payload), timeout=1.0)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._browsers.discard(ws)

    async def control_pump(self):
        """Push the current control state to the drone at a steady rate.

        Rate-driven rather than forwarding each browser message, so the
        drone sees a predictable stream and a browser that stalls cannot
        leave a stale command latched: once input goes quiet the safe
        state is sent instead, and it keeps being sent.
        """
        period = 1.0 / CONTROL_HZ
        while True:
            try:
                if self._drone is not None:
                    ctrl = self._latest_control
                    fresh = (time.monotonic() - self._control_at) <= CONTROL_STALE_AFTER_S
                    if not fresh:
                        ctrl = SAFE_CONTROL

                    # Guidance used to run here. It now runs ON THE DRONE
                    # (drone_agent.py), because the drowning detector that
                    # interrupts it is onboard, and an interrupt that has to
                    # round-trip through the network is no use when the
                    # network is the thing that failed. This loop is purely a
                    # relay of the pilot's input again.
                    self._sent_control = ctrl
                    await self.send_to_drone({"type": "control", "data": ctrl})
            except Exception:
                log.exception("control pump")
            await asyncio.sleep(period)

    @property
    def status(self):
        return {
            "drone_connected": self.drone_connected,
            # Kept under the old name too, so the existing dashboard and
            # any scripts reading /status keep working unchanged.
            "link_connected": self.drone_connected,
            "clients": len(self._browsers),
            "camera_available": self._latest_frame is not None,
            "frames_received": self._frames,
            "latest_control": self._latest_control,
            "sent_control": self._sent_control,
            "latest_telemetry": self._latest_telemetry,
            "mission": self._mission,
            "detection": self._detection,
        }


COOKIE = "seayou_session"

# Reachable without a session. Everything else requires one.
#
# /drone is in here because the AIRCRAFT IS NOT A BROWSER. It has no
# cookie and cannot be sent to a login page, so with logins enabled -
# which is the default - auth_middleware answered the Pi's websocket
# upgrade with a 302 to /login and the drone could never connect at all.
# It went unnoticed because every bench test had been run with --no-auth.
#
# This bypasses the SESSION check, not authentication: drone_ws() checks
# the shared token on every connection before attaching the aircraft.
PUBLIC_PATHS = {"/login", "/logout", "/healthz", "/drone"}

# What the read-only viewer token unlocks, and nothing else.
#
# Listed explicitly rather than "any GET", because the entire point of the
# token is that a link handed to somebody - or baked into a public web app
# - cannot be turned into a way to fly the aircraft. Anything added to this
# set is being made public; think about it before adding one.
VIEWER_PATHS = {"/ws", "/status", "/stream.mjpg", "/mission"}


def _viewer_request(request) -> bool:
    """True if this request presented the read-only viewer token.

    compare_digest rather than ==, so a wrong token cannot be recovered one
    character at a time by timing the replies. The token is only worth
    guessing for a watcher, but this endpoint is the one deliberately
    exposed to the internet, so it gets the careful comparison.
    """
    token = request.app.get("viewer_token")
    if not token:
        return False
    given = request.query.get("viewer", "")
    if not given:
        return False
    return request.path in VIEWER_PATHS and secrets.compare_digest(given, token)


def _cors_headers(origin):
    return {
        "Access-Control-Allow-Origin": origin,
        "Access-Control-Allow-Methods": "GET, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
        # Without this a shared cache could hand one origin's reply, with
        # its Allow-Origin header, to a page on a different origin.
        "Vary": "Origin",
    }


@web.middleware
async def access_middleware(request, handler):
    """Marks read-only viewers, and answers their cross-origin requests.

    ALWAYS installed, including under --no-auth. That is the point: with
    auth off, current_user() hands everyone the pilot role, so without this
    running first the public mirror would quietly inherit it.

    The mirror is served from another origin (the Vercel domain), so its
    fetch() calls are cross-origin and the browser will not show it the
    reply without these headers. They are only added for requests that
    carried the token: the gate is the token, not the origin, and there is
    no cookie on this path for another site to ride on.
    """
    request["viewer_only"] = _viewer_request(request)

    origin = request.headers.get("Origin")
    # Preflight is answered only for a request that already carries the
    # token - a preflight goes to the same URL, query string and all, so
    # there is no reason to hand a CORS promise to an origin that has not
    # presented one. Nothing the mirror actually sends is preflighted
    # anyway: they are plain GETs.
    if request.method == "OPTIONS" and origin and request["viewer_only"]:
        return web.Response(status=204, headers=_cors_headers(origin))

    resp = await handler(request)
    # Not on a prepared response: /ws and /stream.mjpg have already sent
    # their headers by the time the handler returns, and neither needs CORS
    # anyway - websockets are not subject to it and an <img> stream is not
    # read by script.
    if origin and request.get("viewer_only") and not resp.prepared:
        resp.headers.update(_cors_headers(origin))
    return resp


def current_user(request):
    if request.get("viewer_only"):
        # The public mirror. Read-only, no account, and never a pilot -
        # including under --no-auth, which hands everybody else the pilot
        # role. browser_ws() and mission_handler() both gate on this, so
        # this one line is what makes the mirror safe to publish.
        return {"username": "public-viewer", "role": "viewer"}
    users = request.app.get("users")
    if users is None:
        # Auth disabled (--no-auth). Everyone is a pilot; only sane on a
        # trusted LAN, and the startup log says so loudly.
        return {"username": "anonymous", "role": "pilot"}
    return users.verify(request.cookies.get(COOKIE))


@web.middleware
async def auth_middleware(request, handler):
    if (request.path in PUBLIC_PATHS
            or request.get("viewer_only")
            or request.app.get("users") is None):
        return await handler(request)
    if current_user(request):
        return await handler(request)
    # An API call gets a clean 401; a browser gets the login page. Handing
    # a redirect to fetch() would otherwise show up as a confusing parse
    # error rather than "you are logged out".
    if request.path.startswith(("/status", "/mission", "/ws", "/api")):
        return web.json_response({"error": "not authenticated"}, status=401)
    return web.HTTPFound("/login")


async def login(request):
    users = request.app["users"]
    if request.method == "GET":
        return web.Response(text=LOGIN_HTML, content_type="text/html")

    data = await request.post()
    role = users.check(data.get("username", ""), data.get("password", ""))
    if not role:
        log.warning("Failed login for %r from %s",
                    data.get("username"), request.remote)
        return web.Response(
            text=LOGIN_HTML.replace("<!--ERROR-->",
                                    '<p class="err">Wrong username or password.</p>'),
            content_type="text/html", status=401)

    token = users.issue(data.get("username", "").strip().lower(), role)
    resp = web.HTTPFound("/")
    resp.set_cookie(COOKIE, token, httponly=True, samesite="Lax",
                    max_age=12 * 3600)
    log.info("Login: %s (%s)", data.get("username"), role)
    raise resp


async def logout(request):
    resp = web.HTTPFound("/login")
    resp.del_cookie(COOKIE)
    raise resp


async def whoami(request):
    u = current_user(request)
    return web.json_response(u or {"username": None, "role": None})


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

async def drone_ws(request):
    """The endpoint the Pi dials out to."""
    hub = request.app["hub"]
    if hub.token and request.query.get("token") != hub.token:
        log.warning("Drone connection refused: bad token")
        return web.Response(status=401, text="bad token")

    ws = web.WebSocketResponse(heartbeat=5, max_msg_size=16 * 1024 * 1024)
    await ws.prepare(request)
    await hub.attach_drone(ws)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.BINARY:
                # Camera frames arrive as raw JPEG - binary, not base64,
                # which would inflate every frame by a third for nothing.
                hub.on_drone_frame(msg.data)
            elif msg.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if payload.get("type") == "telemetry":
                    await hub.on_drone_telemetry(payload.get("data") or {})
                elif payload.get("type") == "mission_ack":
                    hub.on_mission_ack(payload.get("data") or {})
            elif msg.type == WSMsgType.ERROR:
                log.warning("Drone socket error: %s", ws.exception())
    finally:
        await hub.detach_drone(ws)
    return ws


async def browser_ws(request):
    """Same protocol the dashboard already speaks to the Pi, so the React
    app needs no change beyond pointing at this server."""
    hub = request.app["hub"]
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    user = current_user(request) or {"username": "?", "role": "viewer"}
    role = user.get("role", "viewer")
    hub.add_browser(ws)
    log.info("Dashboard connected: %s (%s) - %d total.",
             user.get("username"), role, len(hub._browsers))
    await ws.send_str(json.dumps({"type": "session", "data": user}))

    # Tell a joining browser where things stand immediately, rather than
    # leaving it to infer the link state from silence.
    await ws.send_str(json.dumps({
        "type": "drone_status", "data": {"connected": hub.drone_connected}
    }))
    if hub._latest_telemetry:
        await ws.send_str(json.dumps({"type": "telemetry", "data": hub._latest_telemetry}))

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                continue
            kind = payload.get("type")
            # Viewers may watch everything and command nothing. Enforced
            # here, on the server, because a browser-side check is only a
            # suggestion - anyone can open a socket and send whatever they
            # like.
            if kind in ("control", "level_cal", "gp_debug") and role != "pilot":
                continue
            if kind == "control":
                hub.set_control(payload.get("data", {}))
                # The stick diagnostics ride along to the drone so they
                # still reach the flight log on the Pi.
                if payload.get("gp"):
                    await hub.send_to_drone({"type": "gp_debug", "gp": payload["gp"]})
            elif kind == "gp_debug":
                await hub.send_to_drone(payload)
            elif kind == "level_cal":
                ok = await hub.send_to_drone(payload)
                await ws.send_str(json.dumps({
                    "type": "level_cal_ack",
                    "data": {"accepted": ok,
                             "reason": "" if ok else "drone not connected"},
                }))
    finally:
        hub.remove_browser(ws)
        log.info("Dashboard disconnected (%d remaining).", len(hub._browsers))
    return ws


async def mjpeg(request):
    """Re-serve the frames the drone sends up, in the same
    multipart/x-mixed-replace form the dashboard already consumes."""
    hub = request.app["hub"]
    if hub._latest_frame is None:
        return web.Response(status=503, text="No video from the drone")
    resp = web.StreamResponse(
        status=200,
        headers={"Content-Type": "multipart/x-mixed-replace; boundary=FRAME"},
    )
    await resp.prepare(request)
    try:
        while True:
            await hub._frame_event.wait()
            frame = hub._latest_frame
            if frame is None:
                break
            await resp.write(
                b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                + frame + b"\r\n"
            )
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def status(request):
    return web.json_response(request.app["hub"].status)


async def mission_handler(request):
    """GET returns mission state; POST starts or aborts one.

        curl -X POST .../mission -d '{"lat":-34.1,"lon":18.47,"alt":5}'
        curl -X POST .../mission -d '{"action":"abort"}'
    """
    hub = request.app["hub"]
    if request.method == "GET":
        return web.json_response(hub._mission)

    u = current_user(request)
    if not u or u.get("role") != "pilot":
        return web.json_response(
            {"ok": False, "message": "you are signed in as a viewer - "
                                     "only pilots can start or abort a mission"},
            status=403)

    try:
        body = await request.json()
    except Exception:
        body = {}

    action = body.get("action")

    if action == "abort":
        return web.json_response(await hub.send_mission({"action": "abort"}))

    if not hub.drone_connected:
        return web.json_response({"ok": False, "message": "drone not connected"}, status=409)

    # Takeoff and landing need the barometer and nothing else - no GPS, no
    # coordinates. That is the whole point of them on this aircraft, whose
    # GPS has never produced a fix. The drone decides whether to accept:
    # it holds the height reading and the limits.
    if action in ("takeoff", "land"):
        payload = {"action": action}
        if action == "takeoff":
            payload["alt"] = body.get("alt")
        result = await hub.send_mission(payload)
        return web.json_response(result, status=200 if result.get("ok") else 409)

    try:
        lat = float(body["lat"]); lon = float(body["lon"]); alt = float(body["alt"])
    except (KeyError, TypeError, ValueError):
        return web.json_response(
            {"ok": False, "message": "need numeric lat, lon and alt"}, status=400)

    result = await hub.send_mission({"lat": lat, "lon": lon, "alt": alt})
    return web.json_response(result, status=200 if result.get("ok") else 409)


def _pilot_is_commanding(ctrl):
    """True if the sticks are off centre or the throttle is up at all."""
    return (
        int(ctrl.get("throttle", 0)) > 0
        or abs(int(ctrl.get("roll", 50)) - 50) > 2
        or abs(int(ctrl.get("pitch", 50)) - 50) > 2
        or abs(int(ctrl.get("yaw", 50)) - 50) > 2
    )


def _file_route(path):
    """Handler serving one fixed file, bound so each route keeps its own
    path rather than sharing the loop variable."""
    async def handler(request):
        return web.FileResponse(path)
    return handler


def build_app(hub, webapp_dir, users=None, viewer_token=None):
    middlewares = [access_middleware]
    if users:
        middlewares.append(auth_middleware)
    app = web.Application(middlewares=middlewares)
    app["users"] = users
    app["hub"] = hub
    app["viewer_token"] = viewer_token
    app.router.add_get("/drone", drone_ws)
    app.router.add_get("/ws", browser_ws)
    app.router.add_get("/stream.mjpg", mjpeg)
    app.router.add_get("/login", login)
    app.router.add_post("/login", login)
    app.router.add_get("/logout", logout)
    app.router.add_get("/whoami", whoami)
    app.router.add_get("/status", status)
    app.router.add_get("/mission", mission_handler)
    app.router.add_post("/mission", mission_handler)

    index = webapp_dir / "index.html"
    if index.is_file():
        app.router.add_get("/", _file_route(index))
        # Registered by walking the build rather than by listing prefixes:
        # hardcoding them means the day someone adds a folder under
        # public/ it silently 404s in production only.
        for child in sorted(webapp_dir.iterdir()):
            if child.is_dir():
                app.router.add_static("/%s/" % child.name, child)
        for f in sorted(webapp_dir.glob("*.*")):
            if f.is_file() and f.name != "index.html":
                app.router.add_get("/%s" % f.name, _file_route(f))
        log.info("Serving the dashboard from %s", webapp_dir)
    else:
        log.warning("No dashboard build at %s - API only. Run 'npm run build' in "
                    "seayou-main and point --webapp at its dist/ folder.", webapp_dir)
    return app


def data_dir() -> Path:
    """Where users.json lives.

    Next to the executable when frozen by PyInstaller, so the accounts
    travel with the app; next to this file otherwise. sys.executable would
    point at python.exe when running from source, which is why the two
    cases are distinguished rather than always using it.
    """
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return Path(__file__).resolve().parent


async def main_async(args):
    hub = Hub(args.token)
    here = Path(__file__).resolve().parent
    if args.webapp:
        webapp = Path(args.webapp)
    elif getattr(sys, "frozen", False):
        # PyInstaller unpacks --add-data into a temp dir it points
        # sys._MEIPASS at. The dashboard rides inside the executable, so
        # the .exe is genuinely self-contained and works from any folder.
        webapp = Path(getattr(sys, "_MEIPASS", here)) / "webapp"
    else:
        webapp = here.parent / "seayou-main" / "dist"

    users = None
    if not args.no_auth:
        users = Users(data_dir() / "users.json")
        if users.empty:
            # A ground station with no accounts would lock everyone out,
            # including the person who just started it. Create one and
            # print the password exactly once.
            pw = secrets.token_urlsafe(9)
            users.add(args.admin_user, pw, "pilot")
            log.warning("=" * 62)
            log.warning("First run - created a pilot account:")
            log.warning("    username: %s", args.admin_user)
            log.warning("    password: %s", pw)
            log.warning("Write this down. It is not shown again.")
            log.warning("=" * 62)

    app = build_app(hub, webapp.resolve(), users, args.viewer_token)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, args.host, args.port).start()

    if users is None:
        log.warning("AUTH DISABLED (--no-auth): anyone who can reach this port "
                    "can fly the aircraft. Trusted networks only.")
    log.info("Ground station on http://%s:%d", args.host, args.port)
    log.info("Drone connects to: ws://<this-pc>:%d/drone%s",
             args.port, "?token=<token>" if args.token else "")
    if args.token:
        log.info("Drone token: %s", args.token)

    if args.viewer_token:
        # Printed as a finished link because the alternative is assembling
        # a URL by hand at the field, and a mistyped token just looks like
        # "the mirror is broken".
        relay = (args.public_url or "").rstrip("/")
        log.info("-" * 62)
        log.info("Public mirror (READ-ONLY) token: %s", args.viewer_token)
        if relay:
            log.info("Live dashboard link:")
            log.info("  %s/?relay=%s&token=%s",
                     args.mirror_app.rstrip("/"), relay, args.viewer_token)
        else:
            log.info("Start a tunnel, then open:")
            log.info("  %s/?relay=<https tunnel url>&token=%s",
                     args.mirror_app.rstrip("/"), args.viewer_token)
        log.info("This token WATCHES ONLY. It cannot arm, fly, or start a mission.")
        log.info("-" * 62)

    await hub.control_pump()


def main():
    ap = argparse.ArgumentParser(description="SeaYou ground station (runs on the PC)")
    ap.add_argument("--host", default="0.0.0.0",
                    help="0.0.0.0 listens on every interface so other devices can reach it")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--webapp", default=None, help="path to the built dashboard (dist/)")
    ap.add_argument("--token", default=None,
                    help="shared secret the drone must present. Generated if omitted; "
                         "pass an empty string to disable (local testing only)")
    ap.add_argument("--viewer-token", default=None,
                    help="read-only token for the public mirror (see "
                         "PUBLIC_MIRROR.md). Generated if omitted; pass an "
                         "empty string to switch the mirror off entirely")
    ap.add_argument("--public-url", default=None,
                    help="the https address a relay (a Cloudflare tunnel, "
                         "say) is serving this ground station on. Only used "
                         "to print a ready-made dashboard link at startup")
    ap.add_argument("--mirror-app", default="https://seayou-indol.vercel.app",
                    help="where the read-only dashboard is deployed")
    ap.add_argument("--no-auth", action="store_true",
                    help="disable logins entirely. Trusted LAN only - it means "
                         "anyone who can reach the port can fly the aircraft")
    ap.add_argument("--admin-user", default="admin",
                    help="username created on first run")
    ap.add_argument("--add-user", nargs=2, metavar=("USERNAME", "PASSWORD"),
                    help="create a viewer account and exit")
    ap.add_argument("--set-role", nargs=2, metavar=("USERNAME", "ROLE"),
                    help="change a role to viewer or pilot, and exit")
    ap.add_argument("--remove-user", metavar="USERNAME",
                    help="delete an account and exit")
    ap.add_argument("--list-users", action="store_true",
                    help="list accounts and exit")
    args = ap.parse_args()

    # Account management runs without starting the server, so it works
    # while a ground station is already up.
    if args.add_user or args.set_role or args.remove_user or args.list_users:
        store = Users(data_dir() / "users.json")
        if args.add_user:
            print(store.add(args.add_user[0], args.add_user[1], "viewer")[1])
        if args.set_role:
            print(store.set_role(args.set_role[0], args.set_role[1])[1])
        if args.remove_user:
            print(store.remove(args.remove_user)[1])
        if args.list_users:
            for u in store.list():
                print(f"  {u['username']:<20} {u['role']}")
            if not store.list():
                print("  (no accounts yet)")
        return

    if args.token is None:
        args.token = secrets.token_urlsafe(16)
    elif args.token == "":
        args.token = None

    if args.viewer_token is None:
        args.viewer_token = secrets.token_urlsafe(12)
    elif args.viewer_token == "":
        args.viewer_token = None
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
