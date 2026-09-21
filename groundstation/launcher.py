"""
SeaYou ground station - the double-click launcher.

This is what SeaYouGroundStation.exe runs. It is a wrapper around
server.py, not a second implementation of it: the Hub, the control pump,
the failsafes and the routes are all server.py's, imported and started
here. The only things this file adds are the ones a one-click launch
needs and a command line does not.

Four of them:

  1. A DRONE TOKEN THAT SURVIVES RESTARTS. server.py generates a fresh
     token every run, which is right for a command line and wrong for
     this: the Pi is configured once, in a systemd unit, and a token that
     changes every launch means the aircraft silently fails to connect
     tomorrow morning. The token is written next to the executable and
     reused. Same for the read-only viewer token the public mirror uses.
  2. THE TUNNEL. Starts cloudflared if it can find it, waits for the
     https address it prints, and assembles the mirror link so it can be
     copied straight out of the window. No tunnel, no problem - the
     ground station runs exactly as it would have.
  3. THE BROWSER. Opens the local dashboard, which is the thing you
     actually fly from.
  4. ONE SUMMARY AT THE END. Startup logging scrolls the important
     things off the top - especially the first-run password - so
     everything needed is reprinted in one block, last, where it is still
     on screen.

Nothing here touches the flight path. If this file failed entirely the
aircraft would be unaffected: it talks to server.py's Hub, which talks to
the Pi, and neither knows the launcher exists.
"""
import argparse
import asyncio
import json
import re
import secrets
import shutil
import socket
import sys
import webbrowser
from pathlib import Path

import server
from auth import Users

CONFIG_NAME = "seayou_station.json"

#: cloudflared prints the address it allocated on stderr, in a box. This is
#: the only line that matters.
TUNNEL_RE = re.compile(rb"https://[a-z0-9-]+\.trycloudflare\.com")

#: How long to wait for that line before giving up on the mirror and
#: carrying on. The ground station does not depend on it.
TUNNEL_WAIT_S = 30


# ---------------------------------------------------------------------
# Config that survives restarts
# ---------------------------------------------------------------------

def load_config() -> dict:
    """Tokens, kept next to the executable.

    Generated once. They have to be stable or the Pi has to be
    reconfigured every time this is launched, which defeats the point of
    a one-click start.
    """
    path = server.data_dir() / CONFIG_NAME
    config = {}
    if path.is_file():
        try:
            config = json.loads(path.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            # A corrupt config must not stop the ground station starting.
            # New tokens mean reconfiguring the Pi once, which is far
            # better than not being able to fly at all.
            print("! %s is unreadable - generating new tokens." % CONFIG_NAME)
            config = {}

    changed = False
    for key, nbytes in (("drone_token", 16), ("viewer_token", 12)):
        if not config.get(key):
            config[key] = secrets.token_urlsafe(nbytes)
            changed = True
    if changed:
        try:
            path.write_text(json.dumps(config, indent=2), encoding="utf-8")
        except OSError as exc:
            print("! could not save %s (%s) - tokens will change next launch."
                  % (CONFIG_NAME, exc))
    return config


def lan_addresses() -> list:
    """Every IPv4 address this machine has, most useful first.

    The Pi has to be pointed at one of these. Which one depends on how it
    is connected, and the Windows hotspot address (usually 192.168.137.1)
    is the one that matters when the drone is on the laptop's hotspot -
    so it is listed first rather than left to be picked out by eye.
    """
    found = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addr = info[4][0]
            if addr not in found and not addr.startswith("127."):
                found.append(addr)
    except socket.gaierror:
        pass
    found.sort(key=lambda a: (not a.startswith("192.168.137."), a))
    return found


# ---------------------------------------------------------------------
# The tunnel
# ---------------------------------------------------------------------

def find_cloudflared() -> str:
    """cloudflared next to the executable, or anywhere on PATH."""
    local = server.data_dir() / ("cloudflared.exe" if sys.platform == "win32"
                                 else "cloudflared")
    if local.is_file():
        return str(local)
    return shutil.which("cloudflared") or ""


async def start_tunnel(port: int):
    """Run cloudflared and return (url, process), or (None, None).

    A quick tunnel needs no account and no configuration, which is why it
    is the one used here. It also gets a DIFFERENT hostname every run,
    which is exactly why the mirror takes its address from the URL at
    runtime instead of having one built in.
    """
    exe = find_cloudflared()
    if not exe:
        return None, None

    print("Starting the public tunnel ...")
    try:
        proc = await asyncio.create_subprocess_exec(
            exe, "tunnel", "--url", f"http://localhost:{port}",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
    except OSError as exc:
        print("! could not start cloudflared (%s) - carrying on without it." % exc)
        return None, None

    async def wait_for_url():
        while True:
            line = await proc.stdout.readline()
            if not line:
                return None
            match = TUNNEL_RE.search(line)
            if match:
                return match.group(0).decode()

    try:
        url = await asyncio.wait_for(wait_for_url(), timeout=TUNNEL_WAIT_S)
    except asyncio.TimeoutError:
        url = None

    if not url:
        print("! the tunnel did not report an address within %ds - carrying on\n"
              "  without the public mirror. The ground station is unaffected."
              % TUNNEL_WAIT_S)
        return None, proc

    # Keep draining, or the pipe fills and cloudflared blocks partway
    # through a flight. Nothing reads the output after this point.
    async def drain():
        while await proc.stdout.readline():
            pass

    asyncio.ensure_future(drain())
    return url, proc


# ---------------------------------------------------------------------
# The summary
# ---------------------------------------------------------------------

def print_summary(port, config, tunnel_url, mirror_app, password, addresses):
    bar = "=" * 70
    print("\n" + bar)
    print("  SeaYou ground station is UP")
    print(bar)

    print("\n  FLY FROM HERE (this machine, full control):")
    print("      http://localhost:%d" % port)
    if addresses:
        print("      http://%s:%d   (from another device on the same network)"
              % (addresses[0], port))

    print("\n  ON THE DRONE (the Pi) - this token no longer changes between")
    print("  launches, so set it once and it keeps working:")
    host = addresses[0] if addresses else "<this-pc>"
    print("      python3 drone_agent.py \\")
    print("          --ground-station ws://%s:%d/drone \\" % (host, port))
    print("          --token %s" % config["drone_token"])
    if len(addresses) > 1:
        print("\n      Other addresses this machine has, if that one is wrong:")
        for a in addresses[1:]:
            print("          %s" % a)

    print("\n  PUBLIC MIRROR (read-only - watches, cannot fly):")
    if tunnel_url:
        print("      %s/?relay=%s&token=%s"
              % (mirror_app.rstrip("/"), tunnel_url, config["viewer_token"]))
        print("\n      Paste that into a browser or send it to someone. It is")
        print("      read-only: no arming, no throttle, no waypoints.")
    else:
        print("      NOT RUNNING - cloudflared was not found.")
        print("      Install it once with:")
        print("          winget install --id Cloudflare.cloudflared")
        print("      or drop cloudflared.exe next to this program, then")
        print("      relaunch. Everything else works without it.")
        print("      Viewer token (for later): %s" % config["viewer_token"])

    if password:
        print("\n  " + "-" * 66)
        print("  FIRST RUN - your login. WRITE THIS DOWN, it is not shown again:")
        print("      username: admin")
        print("      password: %s" % password)
        print("      (lost it? delete users.json next to this program and")
        print("       relaunch - a new account is made)")
        print("  " + "-" * 66)

    print("\n  Close this window to stop the ground station.")
    print(bar + "\n")


# ---------------------------------------------------------------------

async def run(args):
    config = load_config()

    # Create the first account HERE rather than letting server.py do it,
    # so the password can be printed in the summary at the end instead of
    # scrolling off the top during startup - which is how it gets lost.
    password = None
    if not args.no_auth:
        users = Users(server.data_dir() / "users.json")
        if users.empty:
            password = secrets.token_urlsafe(9)
            users.add("admin", password, "pilot")

    server_args = argparse.Namespace(
        host="0.0.0.0",
        port=args.port,
        webapp=args.webapp,
        token=config["drone_token"],
        viewer_token=config["viewer_token"],
        # The launcher prints the mirror link itself, once the tunnel has
        # actually reported an address. server.py cannot know it yet.
        public_url=None,
        mirror_app=args.mirror_app,
        no_auth=args.no_auth,
        admin_user="admin",
    )

    station = asyncio.ensure_future(server.main_async(server_args))

    # Let the server bind before anything is advertised. If it failed -
    # port already taken is the usual one - say so instead of printing a
    # dashboard link that will not open.
    await asyncio.sleep(1.5)
    if station.done():
        exc = station.exception()
        print("\nThe ground station failed to start: %s" % (exc or "unknown error"))
        if isinstance(exc, OSError):
            print("If the port is already in use, another copy is probably")
            print("already running - check for a second window.")
        return 1

    tunnel_url, _ = (None, None) if args.no_mirror else await start_tunnel(args.port)

    print_summary(args.port, config, tunnel_url, args.mirror_app, password,
                  lan_addresses())

    if not args.no_browser:
        try:
            webbrowser.open("http://localhost:%d" % args.port)
        except Exception:
            # No browser is not a reason to take the ground station down.
            pass

    await station
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="SeaYou ground station (double-click launcher)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--webapp", default=None,
                    help="path to a dashboard build, for running from source")
    ap.add_argument("--mirror-app", default="https://seayou-indol.vercel.app",
                    help="where the read-only dashboard is deployed")
    ap.add_argument("--no-mirror", action="store_true",
                    help="do not start the tunnel")
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--no-auth", action="store_true",
                    help="disable logins. Trusted networks only - it means "
                         "anyone who can reach the port can fly the aircraft")
    args = ap.parse_args()

    # Line-buffer stdout. Python block-buffers it whenever it is not a
    # console, so piping this to a log file (which is exactly what you do
    # when something is going wrong) would otherwise show the logging and
    # swallow the summary until the process exits.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass

    try:
        raise SystemExit(asyncio.run(run(args)))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
