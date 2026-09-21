"""u-blox USB GNSS dongle on the Pi, read as NMEA in its own thread.

The GPS used to hang off the Pico and arrive inside the 79-byte telemetry
packet. It does not any more: that module was faulty (proved dead on the
Pi's UART too), the Pi 5's own GPIO UART cannot drive its pins, and the
replacement is a u-blox 7 USB dongle plugged straight into the Pi. So the
position now comes from here and is merged into telemetry by server.py.

Two things this module refuses to do, both for the same reason - a wrong
coordinate is far more dangerous than a missing one, because a wrong one
will fly the aircraft somewhere real:

  * It never reports a position without a fix. No last-known value is
    held over, and 0,0 is never substituted.
  * It never reports a position parsed out of a malformed sentence.
    Checksums are verified before a sentence is believed at all.

Sampling runs in a background thread, exactly like battery.py, because the
30 Hz flight loop must never block on a serial read. The loop takes the
last completed fix, or None.

Port discovery goes through /dev/serial/by-id, never a bare /dev/ttyACM
number: the Pico is also a ttyACM device, and which of the two becomes
ACM0 depends on USB enumeration order at boot. Opening the Pico by mistake
and feeding it NMEA would be a genuinely bad day.
"""

import glob
import logging
import math
import threading
import time

log = logging.getLogger(__name__)

BAUD = 9600

# Substrings that identify the GNSS receiver in its by-id name. The dongle
# fitted enumerates as:
#   usb-u-blox_AG_-_www.u-blox.com_u-blox_7_-_GPS_GNSS_Receiver-if00
_ID_HINTS = ("u-blox", "u_blox", "ublox", "gps", "gnss")

# Anything matching these is NOT the GPS, whatever else it looks like.
_ID_BLOCKLIST = ("pico", "rp2", "2e8a")

# A fix older than this is stale - the dongle has stopped talking, or the
# sky has gone. Reported as no fix rather than as an old position.
FIX_TIMEOUT_S = 5.0

# Trail/speed sanity. A quadcopter that appears to have moved faster than
# this between two fixes did not move - the fix jumped.
MAX_PLAUSIBLE_SPEED_MS = 60.0

EARTH_R_M = 6371000.0


def find_gps_port():
    """The by-id path of the GNSS dongle, or None if one is not plugged in."""
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        name = path.rsplit("/", 1)[-1].lower()
        if any(bad in name for bad in _ID_BLOCKLIST):
            continue
        if any(hint in name for hint in _ID_HINTS):
            return path
    return None


def nmea_checksum_ok(line: str) -> bool:
    """Verify the *hh checksum. A sentence without one is rejected."""
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, given = line[1:].partition("*")
    given = given.strip()
    if len(given) < 2:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def _nmea_to_decimal(value: str, hemi: str, is_lat: bool):
    """ddmm.mmmm + N/S/E/W -> signed decimal degrees, or None.

    Same contract as the Pico-side converter in server.py: return None
    rather than guess.
    """
    if not value or not hemi:
        return None
    hemi = hemi.strip().upper()
    if hemi not in ("N", "S", "E", "W"):
        return None
    deg_len = 2 if is_lat else 3
    if len(value) <= deg_len:
        return None
    try:
        degrees = int(value[:deg_len])
        minutes = float(value[deg_len:])
    except ValueError:
        return None
    if minutes >= 60.0:
        return None
    out = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    if is_lat and not -90.0 <= out <= 90.0:
        return None
    if not is_lat and not -180.0 <= out <= 180.0:
        return None
    return round(out, 7)


def _f(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres. Used for the jump sanity check."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R_M * math.asin(min(1.0, math.sqrt(a)))


class GpsReader:
    """Background NMEA reader. Safe to construct with no dongle plugged in."""

    def __init__(self, port=None, baud: int = BAUD):
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._thread = None
        self._present = False
        # Partial state, owned by the reader thread only. GGA carries the
        # position and satellite count, RMC the ground speed and track, so
        # a complete picture needs both and they arrive separately.
        self._fix = {
            "lat": None, "lon": None, "alt_m": None, "sat_count": 0,
            "hdop": None, "quality": 0, "speed_ms": None, "course_deg": None,
            "utc": None,
        }
        self._last_good = None   # (lat, lon, monotonic-ish timestamp)
        self._sats_in_view = 0
        self._best_snr = 0

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        try:
            import serial  # noqa: F401
        except ImportError:
            log.warning("pyserial not installed - GPS disabled.")
            return
        if self._port is None:
            self._port = find_gps_port()
        if self._port is None:
            log.warning("No GNSS dongle found under /dev/serial/by-id - GPS "
                        "disabled. Plug the u-blox in and restart.")
            return
        self._present = True
        self._thread = threading.Thread(target=self._run, daemon=True, name="gps")
        self._thread.start()
        log.info("GPS reader started on %s at %d baud.", self._port, self._baud)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    @property
    def present(self) -> bool:
        return self._present

    # -- reading -----------------------------------------------------------
    def _run(self):
        import serial

        while not self._stop.is_set():
            ser = None
            try:
                ser = serial.Serial(self._port, self._baud, timeout=1.0)
                log.info("GPS serial open on %s", self._port)
                while not self._stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        self._expire_if_stale()
                        continue
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        self._consume(line)
            except Exception as exc:
                # Unplugging the dongle mid-flight lands here. Publish "no
                # fix" and keep retrying - it may come back.
                log.warning("GPS serial error on %s (%s) - retrying in 2 s",
                            self._port, exc)
                with self._lock:
                    self._latest = self._no_fix("serial_error")
                self._stop.wait(2.0)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass

    def _consume(self, line: str):
        if not nmea_checksum_ok(line):
            return
        parts = line.split("*")[0].split(",")
        # $GPGGA / $GNGGA / $GLGSV - the talker prefix varies, the last
        # three letters are the sentence type.
        sentence = parts[0][3:] if len(parts[0]) >= 6 else parts[0]

        if sentence == "GGA" and len(parts) >= 10:
            quality = int(parts[6]) if parts[6].isdigit() else 0
            lat = _nmea_to_decimal(parts[2], parts[3], True)
            lon = _nmea_to_decimal(parts[4], parts[5], False)
            sats = int(parts[7]) if parts[7].isdigit() else 0
            self._fix["quality"] = quality
            self._fix["sat_count"] = min(sats, 64)
            self._fix["hdop"] = _f(parts[8])
            self._fix["alt_m"] = _f(parts[9])
            self._fix["utc"] = parts[1] or None
            # quality 0 means no fix. The lat/lon fields are empty in that
            # case, but do not rely on it - drop them explicitly.
            if quality > 0 and lat is not None and lon is not None:
                self._publish_position(lat, lon)
            else:
                self._fix["lat"] = None
                self._fix["lon"] = None
                with self._lock:
                    self._latest = self._no_fix("acquiring")

        elif sentence == "RMC" and len(parts) >= 9:
            speed_kn = _f(parts[7])
            self._fix["speed_ms"] = (round(speed_kn * 0.514444, 2)
                                     if speed_kn is not None else None)
            course = _f(parts[8])
            # Course over ground is meaningless standing still and the
            # receiver emits noise for it - withhold it below walking pace.
            if course is not None and (self._fix["speed_ms"] or 0) >= 0.5:
                self._fix["course_deg"] = round(course, 1)
            else:
                self._fix["course_deg"] = None

        elif sentence == "GSV" and len(parts) >= 4:
            # Satellites in view, which is not the same as satellites used.
            # This is the number that climbs first while you stand outside
            # waiting for a cold fix, so it is the one worth watching when
            # there is no position at all yet.
            in_view = int(parts[3]) if parts[3].isdigit() else 0
            self._sats_in_view = min(in_view, 64)
            snrs = []
            for i in range(7, len(parts), 4):
                v = parts[i]
                if v.isdigit():
                    snrs.append(int(v))
            if snrs:
                self._best_snr = max(snrs)
            # While there is no fix, GSV is the only sentence carrying news,
            # so republish on it - otherwise the dashboard's satellite count
            # would sit frozen at whatever the last GGA said and the wait
            # for a cold fix would look like a hang.
            with self._lock:
                if self._latest is None or not self._latest.get("has_fix"):
                    self._latest = self._no_fix("acquiring")

    def _publish_position(self, lat: float, lon: float):
        now = time.time()
        if self._last_good is not None:
            plat, plon, pt = self._last_good
            dt = now - pt
            if dt > 0:
                implied = haversine_m(plat, plon, lat, lon) / dt
                if implied > MAX_PLAUSIBLE_SPEED_MS:
                    log.warning("Discarding GPS jump: %.0f m/s implied", implied)
                    return
        self._last_good = (lat, lon, now)
        self._fix["lat"] = lat
        self._fix["lon"] = lon
        with self._lock:
            self._latest = {
                "source": "pi_usb",
                "has_fix": True,
                "fix": str(self._fix["quality"]),
                "lat": lat,
                "lon": lon,
                "alt_m": self._fix["alt_m"],
                "sat_count": self._fix["sat_count"],
                "sats_in_view": self._sats_in_view,
                "best_snr": self._best_snr,
                "hdop": self._fix["hdop"],
                "speed_ms": self._fix["speed_ms"],
                "course_deg": self._fix["course_deg"],
                "utc": self._fix["utc"],
                "ts": now,
            }

    def _no_fix(self, state: str) -> dict:
        return {
            "source": "pi_usb",
            "has_fix": False,
            "fix": None,
            "lat": None,
            "lon": None,
            "alt_m": None,
            "sat_count": self._fix.get("sat_count", 0),
            "sats_in_view": self._sats_in_view,
            "best_snr": self._best_snr,
            "hdop": self._fix.get("hdop"),
            "speed_ms": None,
            "course_deg": None,
            "state": state,
            "ts": time.time(),
        }

    def _expire_if_stale(self):
        with self._lock:
            last = self._latest
        if last and last.get("has_fix") and \
                time.time() - last.get("ts", 0) > FIX_TIMEOUT_S:
            with self._lock:
                self._latest = self._no_fix("stale")
            self._last_good = None

    # -- consumer ----------------------------------------------------------
    def read(self):
        """Last completed fix, or None when no dongle / nothing read yet."""
        with self._lock:
            if self._latest is None:
                return None
            out = dict(self._latest)
        if out.get("has_fix") and time.time() - out.get("ts", 0) > FIX_TIMEOUT_S:
            return self._no_fix("stale")
        return out


if __name__ == "__main__":
    # Bench check:  python3 gps.py
    logging.basicConfig(level=logging.INFO)
    g = GpsReader()
    g.start()
    if not g.present:
        raise SystemExit("no GNSS dongle found under /dev/serial/by-id")
    try:
        while True:
            time.sleep(1.0)
            print(g.read())
    except KeyboardInterrupt:
        g.stop()
