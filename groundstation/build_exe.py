"""
Build SeaYouGroundStation.exe.

Bundles the launcher, the ground station, the login page, the mission
guidance and the built React dashboard into one Windows executable.

Double-clicking it starts everything: the ground station, the public
tunnel if cloudflared is available, and the dashboard in a browser. The
drone token is written next to the .exe and reused, so the Pi is
configured once rather than every launch - see launcher.py.

    cd seayou-main && npm run build        # the dashboard must be built first
    python groundstation/build_exe.py

Output: groundstation/dist/SeaYouGroundStation.exe

--------------------------------------------------------------------------
How the .exe is meant to be used
--------------------------------------------------------------------------
ONE person runs the .exe - whoever has the drone. It hosts the dashboard
and the drone dials out to it. Everybody else opens it in a browser and
logs in; they do NOT need the .exe.

That is worth being explicit about, because "send the exe to everyone"
sounds like each person runs their own copy, and that does not work: the
drone can only be connected to one ground station at a time. Handing out
the executable is only useful when you want somebody ELSE to be the host.
"""
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
DIST = ROOT / "seayou-main" / "dist"


def main():
    if not (DIST / "index.html").is_file():
        print("ERROR: no dashboard build found at", DIST)
        print("Run this first:   cd seayou-main && npm run build")
        return 1

    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("ERROR: PyInstaller is not installed.   pip install pyinstaller")
        return 1

    # --add-data uses ';' as the separator on Windows and ':' elsewhere.
    sep = ";" if sys.platform == "win32" else ":"

    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--noconfirm", "--clean",
        "--onefile",
        "--name", "SeaYouGroundStation",
        # Console kept on purpose: the first run prints the generated admin
        # password, and the log is the only place a link problem shows up.
        # A windowed build would hide both.
        "--console",
        "--distpath", str(HERE / "dist"),
        "--workpath", str(HERE / "build"),
        "--specpath", str(HERE),
        # The dashboard travels inside the exe and is unpacked at runtime.
        "--add-data", f"{DIST}{sep}webapp",
        # Imported by name from server.py, so PyInstaller's static analysis
        # finds them, but list them explicitly rather than rely on that.
        "--hidden-import", "server",
        "--hidden-import", "mission",
        "--hidden-import", "auth",
        "--hidden-import", "login_page",
        # The launcher is the entry point, not server.py: double-clicking
        # has no command line to pass a token or a tunnel on.
        str(HERE / "launcher.py"),
    ]

    print("Building ...\n  " + " ".join(cmd) + "\n")
    result = subprocess.run(cmd, cwd=str(HERE))
    if result.returncode != 0:
        return result.returncode

    exe = HERE / "dist" / ("SeaYouGroundStation.exe" if sys.platform == "win32"
                           else "SeaYouGroundStation")
    if exe.is_file():
        print(f"\nBuilt: {exe}  ({exe.stat().st_size / 1_048_576:.1f} MB)")
        print("\nFirst run prints a generated admin password - keep it.")
        print("Written next to the .exe, so move them together:")
        print("  users.json           accounts")
        print("  seayou_station.json  the drone and viewer tokens")
        print("\nFor the public mirror, put cloudflared.exe next to the .exe")
        print("(or install it: winget install --id Cloudflare.cloudflared).")
        print("Without it everything else still works.")
    # Tidy the intermediate build tree; the spec file is kept so the build
    # can be reproduced or tweaked.
    shutil.rmtree(HERE / "build", ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
