"""
SeaYou drone dashboard - Pi 5 bridge server.

Replaces the Pi Zero's role (tcp_uart_c) entirely: speaks the exact same
UART wire protocol to the Pico flight controller that tcp_uart_c always
did (the Pico firmware itself, flight_controller.uf2, is untouched), and
exposes control + telemetry over a WebSocket plus the Pi 5's camera as an
MJPEG stream, all served to a browser-based dashboard instead of the
Android app.

Wire protocol to the Pico (unchanged, reverse-engineered from
Tx_Rx_Update_Variables() in fc_quad_gt_cc.cpp - see PROTOCOL.md alongside
this file for the full byte-by-byte layout):

  Control packet, Pi5 -> Pico, 13 bytes:
    [0]='$' [1]=roll [2]=pitch [3]=throttle [4]=yaw
    [5..6]=cmd chars [7..9]=unused [10]=roll_trim [11]=pitch_trim [12]='*'
    roll/pitch/yaw are 0-100, 50=center. throttle is 0-100 (Pico scales
    x10 internally); throttle=0 forces motors off regardless of PID state.
    trims are 0-100, 25=zero offset (degrees = value-25). These are the
    pilot's manual trim and are now added on top of the Pico's stored
    level zero-point, not a replacement for it.
    cmd chars: 'M','G'/'M','N' = GPS hold on/off, 'L','C' = capture the
    level zero-point now, 'L','R' = erase it back to 0/0.

  Telemetry packet, Pico -> Pi5, 63 bytes (59 on firmware built before
  the level zero-point block existed - both are accepted):
    [0]='$' [1..4]=leveled quaternion w,x,y,z as int(q*100)+100
    [5..30]=GPS block [31..32]=status
    [33]=BMP388 temperature as int(C)+100 (0 if barometer not present)
    [34..35]=BMP388 pressure, big-endian uint16, units of 10 Pa (0 if not
    present - the dashboard should treat 0 as "no reading", not 0 Pa)
    [36]=level-capture status: high nibble = state, low nibble = result
    [37..44]=motor_out debug [45..57]=echo of the received control packet
    [58..59]=stored roll zero-point, signed int16 big-endian, 0.01 deg
    [60..61]=stored pitch zero-point, same encoding
    [62]='*'

Run with:
    python3 server.py [--serial /dev/serial0] [--compass-cal /home/pi/compass_cal.txt]
"""
import argparse
import asyncio
import io
import json
import logging
import math
import os
import struct
import subprocess
import time
from pathlib import Path

from aiohttp import web, WSMsgType

log = logging.getLogger("dashboard")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

STATIC_DIR = Path(__file__).parent / "static"

# The built SeaYou React dashboard (seayou-main), if it has been deployed
# here. When present it takes over "/" and the original pilot dashboard
# moves to "/pilot"; when absent the pilot dashboard keeps "/" exactly as
# before. That fallback is deliberate - the pilot dashboard is the
# flight-tested one, and the drone must never be left with no usable UI
# because a front-end build went missing or was deployed half-finished.
WEBAPP_DIR = Path(__file__).parent / "webapp"

# ---------------------------------------------------------------------------
# Wire protocol - pack/unpack, kept in exact lockstep with the Pico firmware.
# ---------------------------------------------------------------------------

CTRL_LEN = 13
# The telemetry packet grew from 59 to 63 bytes when the Pico gained the
# level zero-point block. Both lengths are accepted (see
# _read_telemetry_with_timeout), so this server works against a Pico that
# hasn't been reflashed yet - that shows up as a missing level panel
# rather than a dead link.
TELEM_LEN = 81
TELEM_LEN_LEGACY = 59
# Every packet length this server understands, longest first. 65 adds the
# I2C fault counter, 63 added the level zero-point block, 59 is the
# original. Checked longest-first so a shorter layout can never be matched
# by accident inside a longer packet.
# 81 adds the VSYS rail voltage in bytes 78-79. Longest first, so a
# shorter layout can never be matched by accident inside a longer packet.
TELEM_LENS = (81, 79, 77, 65, 63, 59)
START_BYTE = ord('$')
END_BYTE = ord('*')

# Command character pairs the Pico understands in control bytes [5..6].
# Held in the outgoing packet for LEVEL_CMD_HOLD_S so the Pico is certain
# to see one; its own edge latch makes sure a held command only fires once.
LEVEL_COMMANDS = {
    "start": (ord('L'), ord('C')),
    "reset": (ord('L'), ord('R')),
}

# Mirrors the LEVEL_CAL_* / LEVEL_RES_* defines in fc_quad_gt_cc.cpp.
LEVEL_STATES = {0: "idle", 1: "settling", 2: "sampling"}
LEVEL_RESULTS = {
    0: None,
    1: "ok",
    2: "moved",
    3: "out_of_range",
    4: "throttle_not_zero",
}

# Safe idle state: centered sticks, zero throttle (Pico forces motors off
# when throttle==0 regardless of anything else - see ctrl_channel[2]==0
# handling in fc_quad_gt_cc.cpp). This is what gets sent whenever the
# browser isn't actively connected and sending fresh input.
from battery import BatteryMonitor
from gps import GpsReader

SAFE_CONTROL = {
    "roll": 50, "pitch": 50, "throttle": 0, "yaw": 50,
    "cmd0": 0, "cmd1": 0, "roll_trim": 25, "pitch_trim": 25,
}


def pack_control(c: dict) -> bytes:
    pkt = bytearray(CTRL_LEN)
    pkt[0] = START_BYTE
    pkt[1] = _clamp_byte(c.get("roll", 50))
    pkt[2] = _clamp_byte(c.get("pitch", 50))
    pkt[3] = _clamp_byte(c.get("throttle", 0))
    pkt[4] = _clamp_byte(c.get("yaw", 50))
    pkt[5] = c.get("cmd0", 0) & 0xFF
    pkt[6] = c.get("cmd1", 0) & 0xFF
    pkt[7] = 0
    pkt[8] = 0
    pkt[9] = 0
    pkt[10] = _clamp_byte(c.get("roll_trim", 25))
    pkt[11] = _clamp_byte(c.get("pitch_trim", 25))
    pkt[12] = END_BYTE
    return bytes(pkt)


def _clamp_byte(v) -> int:
    try:
        v = int(v)
    except (TypeError, ValueError):
        v = 0
    return max(0, min(255, v))


def unpack_telemetry(pkt: bytes) -> dict | None:
    if len(pkt) not in TELEM_LENS:
        return None
    if pkt[0] != START_BYTE or pkt[-1] != END_BYTE:
        return None

    qw = (pkt[1] - 100) / 100.0
    qx = (pkt[2] - 100) / 100.0
    qy = (pkt[3] - 100) / 100.0
    qz = (pkt[4] - 100) / 100.0
    roll, pitch, yaw = _quat_to_euler_deg(qw, qx, qy, qz)

    fix_type_byte = pkt[26]
    fix_char = chr(fix_type_byte) if 32 <= fix_type_byte < 127 else None
    has_fix = fix_char in ("2", "3")

    telemetry = {
        "attitude": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "gps": {
            "fix": fix_char,
            "has_fix": has_fix,
            # Reported even without a fix. It used to be forced to 0
            # unless has_fix, which made the number useless for the one
            # thing you actually want it for: watching satellites climb
            # while you WAIT for a fix. Sanity-capped because without a
            # fix this byte can hold whatever the GPS block last had.
            "sat_count": pkt[27] if pkt[27] <= 64 else 0,
            # Raw NMEA-format (DDMM.MMMM / DDDMM.MMMM) ASCII fields, exactly
            # as the GPS emits them.
            "lat_raw": bytes(pkt[5:15]).decode("ascii", errors="replace") if has_fix else None,
            "lon_raw": bytes(pkt[15:26]).decode("ascii", errors="replace") if has_fix else None,
            # Decimal degrees. Nothing can navigate to a coordinate from the
            # raw NMEA text, so this conversion is the prerequisite for any
            # fly-to-waypoint feature. None whenever there is no fix or the
            # field is malformed - callers must treat None as "position
            # unknown" and never as 0,0, which is a real place in the
            # Atlantic.
            "lat": _nmea_to_decimal(bytes(pkt[5:15]).decode("ascii", errors="replace"), True)
                   if has_fix else None,
            "lon": _nmea_to_decimal(bytes(pkt[15:26]).decode("ascii", errors="replace"), False)
                   if has_fix else None,
        },
        "baro": _unpack_baro(pkt),
        "rail": _unpack_rail(pkt),
        "level": _unpack_level(pkt),
        # Bounded-out I2C transactions since the Pico booted. None on
        # firmware predating the timeout fix. A rising count with the link
        # still alive means the bus glitched and the flight loop survived
        # it - which is exactly what the fix is for.
        "i2c_faults": struct.unpack(">H", pkt[62:64])[0] if len(pkt) >= 65 else None,
        # Did the boot-time gyro bias sweep succeed? False means the sweep
        # was rejected (the drone moved during it), so there is NO bias
        # correction and the rate loop will drift to one side. Power-cycle
        # the Pico with the drone sitting still to retry. None on firmware
        # that predates the flag.
        "gyro_cal_ok": bool(pkt[36] & 0x80) if len(pkt) >= 63 else None,
        # Has the EKF quaternion ever gone NaN and been reinitialised?
        # Should be False forever. True means the filter was fed something
        # bad and recovered - worth investigating, not worth flying on.
        "ekf_nan": bool(pkt[36] & 0x40) if len(pkt) >= 63 else None,
        # DIAGNOSTIC: raw accelerometer, in LSB. accel_raw is the sampled
        # instant; accel_peak is the per-axis peak |value| since the last
        # packet, which is what actually reveals clipping (the 30 Hz sample
        # aliases vibration). At +/-16g the rail is 32767.
        "accel_raw": [struct.unpack(">h", pkt[64+i*2:66+i*2])[0] for i in range(3)] if len(pkt) >= 77 else None,
        "accel_peak": [struct.unpack(">H", pkt[70+i*2:72+i*2])[0] for i in range(3)] if len(pkt) >= 77 else None,
        # DIAGNOSTIC: |magnetometer| in uT. Should be constant - the earth
        # field does not change. If it moves with throttle, motor current is
        # the source.
        "mag_ut": struct.unpack(">H", pkt[76:78])[0] / 10.0 if len(pkt) >= 79 else None,
        # TEMPORARY debug readout of the Pico's actual computed motor_out[]
        # values (0-1000 each) - see the matching comment in PWM_Write() /
        # Tx_Rx_Update_Variables() in fc_quad_gt_cc.cpp. Remove both sides
        # together once the motor-spin issue is resolved.
        "motor_out_debug": [
            (pkt[37 + i * 2] << 8) | pkt[37 + i * 2 + 1] for i in range(4)
        ],
        # TEMPORARY: raw echo of the 13 bytes the Pico actually received,
        # to compare byte-for-byte against what pack_control() sent.
        "rx_echo_debug": list(pkt[45:58]),
        "ts": time.time(),
    }
    return telemetry


def _nmea_to_decimal(raw: str, is_lat: bool) -> float | None:
    """NMEA ddmm.mmmm[N/S] / dddmm.mmmm[E/W] -> signed decimal degrees.

    Returns None rather than guessing whenever the field is short, padded
    with junk, or otherwise not a position. A wrong coordinate is far worse
    than a missing one: it would send a drone somewhere real.
    """
    if not raw:
        return None
    raw = raw.strip().replace(chr(0), "").strip()
    if len(raw) < 4:
        return None
    hemi = raw[-1].upper()
    if hemi not in ("N", "S", "E", "W"):
        return None
    body = raw[:-1]
    deg_len = 2 if is_lat else 3
    if len(body) <= deg_len:
        return None
    try:
        degrees = int(body[:deg_len])
        minutes = float(body[deg_len:])
    except ValueError:
        return None
    if minutes >= 60.0:
        return None
    value = degrees + minutes / 60.0
    if hemi in ("S", "W"):
        value = -value
    if is_lat and not -90.0 <= value <= 90.0:
        return None
    if not is_lat and not -180.0 <= value <= 180.0:
        return None
    return round(value, 7)


def _unpack_level(pkt: bytes) -> dict | None:
    """Level zero-point block. Only present on firmware built with
    Level_Capture_Run() (63-byte packets); older firmware reports None and
    the dashboard hides the panel rather than showing zeros that aren't
    really coming from anywhere."""
    if len(pkt) < 63:
        return None
    # 0x03, not 0x0F: bit 7 of this byte now carries the gyro-calibration
    # flag, and masking too wide would read that back as a bogus level
    # state of 8+.
    state = (pkt[36] >> 4) & 0x03
    result = pkt[36] & 0x0F
    roll_off = struct.unpack(">h", pkt[58:60])[0] / 100.0
    pitch_off = struct.unpack(">h", pkt[60:62])[0] / 100.0
    return {
        "state": LEVEL_STATES.get(state, "unknown"),
        "busy": state != 0,
        "result": LEVEL_RESULTS.get(result, "unknown"),
        "roll_offset_deg": roll_off,
        "pitch_offset_deg": pitch_off,
    }


def _unpack_rail(pkt: bytes) -> dict | None:
    """VSYS, the drone's 5 V rail, measured by the Pico's own ADC.

    Per V4_Wiring_Diagram.png this is the same rail that feeds the GPS
    module's VCC and the I2C sensors, so it is not just a Pico health
    figure - it says whether those parts have a supply at all.

    ~4.8 V means the rail is only being backfed from USB through the Pico's
    Schottky diode. 5.0 V or above means the ESC BEC is genuinely supplying
    it. Only firmware that sends 81-byte packets has this.
    """
    if len(pkt) < 81:
        return None
    mv = (pkt[78] << 8) | pkt[79]
    if mv == 0:
        return None
    volts = mv / 1000.0
    return {
        "vsys_v": round(volts, 3),
        # Named rather than left to the dashboard to guess at.
        "source": "esc_bec" if volts >= 4.95 else "usb_backfeed",
    }


def _unpack_baro(pkt: bytes) -> dict | None:
    press_scaled = (pkt[34] << 8) | pkt[35]
    if press_scaled == 0:
        # 0 is what the Pico sends when no BMP388 was detected at init -
        # a real reading is never exactly 0 Pa, so this is an unambiguous
        # "not present" marker rather than a valid low-pressure value.
        return None
    return {
        "temp_c": pkt[33] - 100,
        "pressure_hpa": press_scaled / 10.0,
    }


def _quat_to_euler_deg(w, x, y, z):
    # Mirrors get_error_angles_from_Quaternion() in fc_quad_gt_cc.cpp exactly,
    # so the dashboard shows the same numbers the flight controller computes.
    pitch = 57.2958 * math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    roll = 57.2958 * math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = 57.2958 * math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Flight logging.
# ---------------------------------------------------------------------------

def _gp_fields(gp) -> list:
    """Exactly 11 CSV fields: 4 raw axes, 4 centres, 3 resulting commands.

    Always 11, whatever arrives. The payload comes from a browser, so a
    malformed or truncated one must not shift every later column - a
    corrupted CSV would cost more than the data is worth.
    """
    def four(v):
        if not isinstance(v, list):
            return ["", "", "", ""]
        out = [x if isinstance(x, (int, float)) else "" for x in v[:4]]
        return out + [""] * (4 - len(out))

    def three(v):
        if not isinstance(v, list):
            return ["", "", ""]
        out = [x if isinstance(x, (int, float)) else "" for x in v[:3]]
        return out + [""] * (3 - len(out))

    if not isinstance(gp, dict):
        return [""] * 11
    return four(gp.get("ax")) + four(gp.get("c")) + three(gp.get("out"))


class FlightLogger:
    """Writes every telemetry cycle to a CSV so a flight can be analysed
    afterwards instead of reasoned about from memory or video frames.

    Deliberately dumb and continuous: one file per service start, logging
    whenever telemetry is flowing rather than trying to detect "a flight".
    A clever start/stop heuristic that mis-fires loses the one takeoff you
    needed. A flight_id column marks each throttle-up instead, so the
    interesting rows are still easy to pick out.

    Nothing in here is allowed to raise into the link loop - a logging
    problem must never cost you the control link.
    """

    FLUSH_EVERY_S = 0.5   # push to the OS - survives the process being killed
    FSYNC_EVERY_S = 2.0   # push to the card - survives power being cut

    COLUMNS = [
        "t", "wall_clock", "flight_id", "link",
        "throttle_cmd", "ctrl_roll", "ctrl_pitch", "ctrl_yaw",
        "roll", "pitch", "yaw",
        "m0", "m1", "m2", "m3", "m_min", "m_max", "m_spread",
        "sat_high", "sat_low",
        "i2c_faults", "gyro_cal_ok", "ekf_nan", "level_roll_off", "level_pitch_off",
        "baro_height_m", "sats",
        # RAW gamepad axes and the resting centre the browser is subtracting.
        # ctrl_roll=50 alone cannot distinguish "pilot centred the stick" from
        # "the stick is broken and cannot produce anything else" - these can.
        "gp_ax0", "gp_ax1", "gp_ax2", "gp_ax3",
        "gp_c0", "gp_c1", "gp_c2", "gp_c3",
        # What those axes WOULD command, even when control has not been
        # taken (in which case ctrl_* logs the safe 50 regardless and tells
        # you nothing about the sticks). Lets a bench test be read straight
        # out of the log: hardware -> centring -> deadzone -> wire byte.
        "gp_out_roll", "gp_out_pitch", "gp_out_yaw",
        # Position from the Pi's USB GNSS dongle. Appended at the end so
        # existing logs and anything reading these files by column name
        # keep working. Blank whenever there is no fix - never 0,0.
        "lat", "lon", "gps_alt_m", "gps_speed_ms", "gps_course_deg",
    ]

    def __init__(self, log_dir: str):
        self.dir = Path(log_dir)
        self._f = None
        self._path = None
        self._t0 = None
        self._flight_id = 0
        self._was_throttled = False
        self._zero_since = None
        self._last_flush = 0.0
        self._last_fsync = 0.0
        self._failed = False

    def _open(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        # The Pi may have no network time in a field, so the wall clock can
        # be wrong. Keep it in the name for convenience but never rely on it
        # for ordering - the 't' column is monotonic seconds since start.
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._path = self.dir / f"flight_{stamp}.csv"
        self._f = open(self._path, "w", buffering=1)
        self._f.write(",".join(self.COLUMNS) + "\n")
        self._t0 = time.monotonic()
        log.info("Flight log: %s", self._path)

    def log(self, telemetry: dict, control: dict, connected: bool, gp=None):
        if self._failed:
            return
        try:
            if self._f is None:
                self._open()

            now = time.monotonic()
            throttle = int(control.get("throttle", 0) or 0)

            # New flight_id on each throttle-up after a few seconds at idle.
            if throttle > 0:
                if not self._was_throttled and (
                        self._zero_since is None or now - self._zero_since > 3.0):
                    self._flight_id += 1
                    log.info("Flight %d starting (throttle up)", self._flight_id)
                self._was_throttled = True
                self._zero_since = None
            else:
                if self._was_throttled:
                    self._zero_since = now
                self._was_throttled = False

            att = telemetry.get("attitude") or {}
            lvl = telemetry.get("level") or {}
            baro = telemetry.get("baro") or {}
            gps = telemetry.get("gps") or {}
            m = telemetry.get("motor_out_debug") or [None] * 4

            if all(v is not None for v in m):
                m_min, m_max = min(m), max(m)
                spread = m_max - m_min
                # Saturation is the thing to look for: a motor pinned at the
                # 1000 ceiling or the 50 floor has no authority left to
                # correct with, whichever way the mixer signs point.
                #
                # Only meaningful under power - at zero throttle the firmware
                # drives all four to 0 deliberately, which would otherwise
                # flag "floored" on every idle row and bury the real events.
                if throttle > 0:
                    sat_hi = int(m_max >= 1000)
                    sat_lo = int(m_min <= 50)
                else:
                    sat_hi = sat_lo = 0
            else:
                m_min = m_max = spread = sat_hi = sat_lo = ""

            row = [
                "%.3f" % (now - self._t0),
                time.strftime("%H:%M:%S"),
                self._flight_id,
                int(bool(connected)),
                throttle,
                control.get("roll", ""), control.get("pitch", ""), control.get("yaw", ""),
                "%.2f" % att.get("roll", 0.0),
                "%.2f" % att.get("pitch", 0.0),
                "%.2f" % att.get("yaw", 0.0),
                m[0], m[1], m[2], m[3], m_min, m_max, spread,
                sat_hi, sat_lo,
                telemetry.get("i2c_faults", ""),
                "" if telemetry.get("gyro_cal_ok") is None else int(telemetry["gyro_cal_ok"]),
                "" if telemetry.get("ekf_nan") is None else int(telemetry["ekf_nan"]),
                lvl.get("roll_offset_deg", ""), lvl.get("pitch_offset_deg", ""),
                "%.2f" % baro["height_m"] if baro.get("height_m") is not None else "",
                gps.get("sat_count", ""),
                *_gp_fields(control.get("_gp") or gp),
                "%.7f" % gps["lat"] if gps.get("lat") is not None else "",
                "%.7f" % gps["lon"] if gps.get("lon") is not None else "",
                gps.get("alt_m") if gps.get("alt_m") is not None else "",
                gps.get("speed_ms") if gps.get("speed_ms") is not None else "",
                gps.get("course_deg") if gps.get("course_deg") is not None else "",
            ]
            self._f.write(",".join(str(c) for c in row) + "\n")

            if now - self._last_flush >= self.FLUSH_EVERY_S:
                self._f.flush()
                self._last_flush = now
            if now - self._last_fsync >= self.FSYNC_EVERY_S:
                os.fsync(self._f.fileno())
                self._last_fsync = now

        except Exception:
            # Log once and then stay quiet - a broken logger must not spam
            # the link loop 30 times a second, nor stop the drone flying.
            self._failed = True
            log.exception("Flight logging failed - continuing without it")


# ---------------------------------------------------------------------------
# UART bridge to the Pico.
# ---------------------------------------------------------------------------

class PicoLink:
    """Owns the serial port, does the calibration handshake once at start,
    then continuously exchanges control/telemetry packets at a fixed rate.
    Broadcasts telemetry to whatever WebSocket clients are connected, and
    always sends SAFE_CONTROL whenever no client has sent a fresh command
    recently - this is a second, independent safety net on top of the
    Pico's own 1-second no-data failsafe (Is_uart0_receiving_and_Action()).
    """

    UPDATE_HZ = 30
    REPLY_TIMEOUT_S = 0.05
    CONTROL_STALE_AFTER_S = 0.5
    LEVEL_CMD_HOLD_S = 0.5

    def __init__(self, serial_path: str, baud: int, compass_cal_path: str,
                 log_dir: str = "/home/pi/flightlogs"):
        self.flight_log = FlightLogger(log_dir)
        self.serial_path = serial_path
        self.baud = baud
        self.compass_cal_path = compass_cal_path
        self._serial = None
        self._latest_control = dict(SAFE_CONTROL)
        self._control_updated_at = 0.0
        self._clients: set[web.WebSocketResponse] = set()
        self._latest_telemetry: dict | None = None
        self._connected = False
        # Captured from the first real barometer reading after this service
        # (re)starts - "height" is always relative to that, not true sea
        # level, since the BMP388 only gives absolute pressure.
        self._baseline_pressure_hpa: float | None = None
        # Pending level zero-point command, held in the outgoing packet's
        # cmd bytes for LEVEL_CMD_HOLD_S then dropped.
        self._level_cmd: tuple[int, int] | None = None
        self._level_cmd_until = 0.0
        # Latest raw-gamepad diagnostics. Kept STRICTLY apart from
        # _latest_control: this arrives whether or not a client has taken
        # control, and must never influence what is sent to the Pico. It
        # exists only so the flight log records what the sticks actually
        # reported.
        self._gp_debug = None
        # Consecutive control-packet writes the Pico refused to accept.
        self._write_stalls = 0
        self._serial_timeout_exc: type[BaseException] = TimeoutError
        # Replaced with pyserial's real classes in run(). OSError is the
        # honest stand-in until then: every pyserial error derives from it.
        self._serial_error_exc: type[BaseException] = OSError
        self._serial_module = None
        # INA219 pack monitor on the Pi's own I2C bus. Samples on its
        # own thread; this loop only ever reads the last value, so a
        # stuck I2C bus cannot stall the 30 Hz link loop.
        self.battery = BatteryMonitor()
        self.battery.start()
        # u-blox USB GNSS dongle, also on its own thread and also read
        # without blocking. The Pico's GPS block is dead hardware, so
        # this is where position actually comes from now.
        self.gps = GpsReader()
        self.gps.start()

    def _select_control(self, control: dict) -> dict:
        """Last chance to change what gets sent this cycle.

        Returns its argument untouched. Subclasses override it to inject
        autonomous control; keeping it here rather than in the subclass
        means the flight loop stays in one place with one set of failsafes
        around it.
        """
        return control

    def set_control(self, control: dict, gamepad: dict | None = None):
        merged = dict(SAFE_CONTROL)
        merged.update(control)
        # Underscore-prefixed so pack_control() ignores it - this rides along
        # for the flight log only and must never reach the wire packet.
        merged["_gp"] = gamepad
        self._latest_control = merged
        self._control_updated_at = time.monotonic()

    def set_gamepad_debug(self, gp):
        """Record the browser's raw stick values for the flight log.

        Diagnostics only - deliberately does NOT go near _latest_control,
        so an idle dashboard reporting its sticks can never disturb the
        pilot's input.
        """
        self._gp_debug = gp if isinstance(gp, dict) else None

    def request_level_cal(self, action: str) -> dict:
        """Ask the Pico to capture (or erase) its level zero-point.

        The measurement itself has to happen on the Pico: the telemetry
        packet encodes each quaternion component as int(q*100)+100, about
        1.15 degrees per step, so there is no way to level anything to a
        useful accuracy from this end of the link. All this does is hold
        the command in the control packet long enough to be seen.
        """
        if action not in LEVEL_COMMANDS:
            return {"accepted": False, "reason": f"unknown action {action!r}"}
        if int(self._latest_control.get("throttle", 0)) != 0:
            return {"accepted": False, "reason": "throttle must be at zero"}
        self._level_cmd = LEVEL_COMMANDS[action]
        self._level_cmd_until = time.monotonic() + self.LEVEL_CMD_HOLD_S
        log.info("Level zero-point '%s' requested.", action)
        return {"accepted": True, "action": action}

    def register(self, ws: web.WebSocketResponse):
        self._clients.add(ws)

    def unregister(self, ws: web.WebSocketResponse):
        self._clients.discard(ws)
        if not self._clients:
            # No dashboard connected at all - fall back to safe state
            # immediately rather than waiting on the staleness timeout.
            self._latest_control = dict(SAFE_CONTROL)

    @property
    def status(self) -> dict:
        return {
            "link_connected": self._connected,
            "clients": len(self._clients),
        }

    def _apply_relative_height(self, telemetry: dict):
        baro = telemetry.get("baro")
        if not baro:
            return
        if self._baseline_pressure_hpa is None:
            self._baseline_pressure_hpa = baro["pressure_hpa"]
        # Standard barometric formula, relative to the baseline pressure
        # rather than sea level - this is "height since the baseline was
        # captured" (first reading after this service started), not a
        # true altitude above sea level.
        ratio = baro["pressure_hpa"] / self._baseline_pressure_hpa
        baro["height_m"] = 44330.0 * (1.0 - ratio ** (1.0 / 5.255))

    async def _reopen_serial(self) -> bool:
        """Reopen /dev/pico after the device went away and came back.

        Power-cycling the Pico is a NORMAL operation on this aircraft - the
        handover tells you to do it whenever the flight controller is stuck
        waiting for calibration. But USB does not preserve the file
        descriptor across that: the node disappears and returns (often as a
        different ttyACM number, which is why /dev/pico is a udev symlink),
        and every write to the old handle then fails with Errno 5.

        Without this the loop caught that error, logged it, and tried the
        same dead handle again thirty times a second forever - the link
        never came back until somebody restarted the service by hand.
        Observed 2026-09-21 after a power cycle.
        """
        try:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass  # It is already broken; closing is best-effort.
            self._serial = self._serial_module.Serial(
                self.serial_path, self.baud, timeout=0, write_timeout=0.25
            )
            log.info("Reopened %s - the Pico is back.", self.serial_path)
            self._connected = True
            self._write_stalls = 0
            return True
        except Exception as exc:
            # Expected while the Pico is still enumerating. Logged at most
            # once a second by the caller's backoff, not 30 times.
            log.warning("Cannot reopen %s yet (%s)", self.serial_path, exc)
            self._connected = False
            return False

    async def run(self):
        import serial  # pyserial - deferred import so the rest of the
        # server (dashboard, video) still works if pyserial isn't installed.

        log.info("Opening %s @ %d baud", self.serial_path, self.baud)
        # write_timeout matters as much as timeout. Without it pyserial
        # blocks forever if the Pico stops draining its USB CDC endpoint -
        # which is exactly what a hung flight controller looks like. That
        # block happens inside the asyncio loop, so it freezes the ENTIRE
        # web server too: no telemetry, no page loads, and no working
        # DISARM button, with the process still "active" under systemd and
        # nothing logged. Observed for real on 2026-08-31. A timeout turns
        # that into a logged error with the UI still responsive.
        self._serial = serial.Serial(
            self.serial_path, self.baud, timeout=0, write_timeout=0.25
        )
        self._connected = True
        # Kept so the link loop can tell "the Pico stopped draining" apart
        # from a genuine bug without importing pyserial at module scope.
        self._serial_timeout_exc = serial.SerialTimeoutException
        self._serial_error_exc = serial.SerialException
        self._serial_module = serial

        # Never let a handshake problem kill the service. Calibration is a
        # nice-to-have; the control link and the DISARM button are not.
        try:
            await self._wait_for_calibration()
        except Exception:
            log.exception("Calibration handshake raised - continuing to the control "
                          "loop anyway")

        period = 1.0 / self.UPDATE_HZ
        while True:
            cycle_start = time.monotonic()

            # This loop is the entire Pi<->Pico link - control out, telemetry
            # in. An uncaught exception anywhere in here previously killed
            # the whole coroutine silently (nothing retrieves an asyncio
            # task's exception unless something awaits it), which stopped
            # all communication with the Pico permanently while the rest of
            # the web server kept responding normally, with no error logged
            # anywhere. Catch broadly, log loudly, and keep looping - a
            # flight-control link going silently dead is far worse than one
            # noisy log line.
            try:
                control = self._latest_control
                if (time.monotonic() - self._control_updated_at) > self.CONTROL_STALE_AFTER_S:
                    control = SAFE_CONTROL

                # Hook for onboard autonomy. Identity here - server.py's own
                # behaviour is unchanged - but drone_agent.py overrides it to
                # run waypoint guidance and the drowning-detection interrupt
                # ON THE PI, inside this same 30 Hz loop, so neither depends
                # on the network being up.
                control = self._select_control(control)

                if self._level_cmd is not None:
                    if time.monotonic() < self._level_cmd_until:
                        # Copy first - control may be the module-level
                        # SAFE_CONTROL dict, which must never be mutated.
                        control = dict(control)
                        control["cmd0"], control["cmd1"] = self._level_cmd
                    else:
                        self._level_cmd = None

                try:
                    self._serial.write(pack_control(control))
                    if self._write_stalls:
                        log.info("Pico is draining the link again after %d stalled "
                                 "write(s).", self._write_stalls)
                        self._write_stalls = 0
                        self._connected = True
                except self._serial_error_exc as exc:
                    # The DEVICE went away - a power cycle, or the cable.
                    # Not the same as the Pico refusing to read: this handle
                    # is dead and will never work again, so get a new one.
                    log.warning("Pico serial write failed (%s) - reopening %s",
                                exc, self.serial_path)
                    self._connected = False
                    await self._reopen_serial()
                    await asyncio.sleep(0.5)
                    continue
                except self._serial_timeout_exc:
                    # The Pico is not reading its end of the USB CDC pipe -
                    # it has hung, or been unplugged. Log the first one and
                    # then roughly once a second, rather than 30x a second.
                    self._write_stalls += 1
                    if self._write_stalls == 1 or self._write_stalls % self.UPDATE_HZ == 0:
                        log.error("Pico is not accepting data (%d stalled writes). It has "
                                  "most likely hung - the web UI stays up, but there is no "
                                  "control link. Power-cycle or reflash the Pico.",
                                  self._write_stalls)
                    self._connected = False
                    await asyncio.sleep(0.05)
                    continue

                telemetry = await self._read_telemetry_with_timeout()
                if telemetry:
                    self._apply_relative_height(telemetry)
                    # Non-blocking: last completed INA219 sample, or
                    # None when no sensor is fitted.
                    batt = self.battery.read()
                    if batt is not None:
                        telemetry["battery"] = batt
                    # The dongle on the Pi replaces the Pico's GPS block
                    # outright rather than merging with it: the module
                    # wired to the Pico is faulty, so those bytes are not
                    # a second opinion worth blending in. Only replace
                    # when the reader has actually produced something -
                    # with no dongle plugged in, whatever the Pico sent
                    # stays, and old flight logs still parse.
                    fix = self.gps.read()
                    if fix is not None:
                        telemetry["gps"] = fix
                    self._latest_telemetry = telemetry
                    # Logged with the control packet that produced it, so a
                    # row shows both what was asked for and what came back.
                    self.flight_log.log(telemetry, control, self._connected, self._gp_debug)
                    await self._broadcast(telemetry)
            except Exception:
                log.exception("Unexpected error in the Pico link loop - continuing")

            elapsed = time.monotonic() - cycle_start
            await asyncio.sleep(max(0.0, period - elapsed))

    async def _wait_for_calibration(self):
        # A single one-shot attempt right at startup is fragile: the Pico
        # takes several seconds to boot (a 2s sleep plus multiple init
        # steps before it even sets up UART0), so whether one attempt lands
        # inside that window depends on exactly when the Pico happens to
        # have been power-cycled relative to when this service started -
        # not something worth relying on, especially since someone may
        # power-cycle the Pico *after* this service is already running
        # (e.g. mid-demo, after a crash). Retry indefinitely instead, so
        # the two can happen in either order and this just catches up.
        cal_path = Path(self.compass_cal_path)
        if not cal_path.exists():
            log.warning("Compass cal file %s not found - Pico will stay locked until "
                        "something sends it 100 calibration bytes.", cal_path)
            return

        payload = cal_path.read_bytes()[:100].ljust(100, b"\x00")
        # Bounded, not infinite: the Pico's control-packet reader now
        # resyncs on '$' (see USB_CDC_Read() in fc_quad_gt_cc.cpp), which
        # means if the Pico is already calibrated from an earlier boot, this
        # calibration payload - plain comma-separated decimal text, which
        # essentially never contains a literal '$' byte - just gets silently
        # discarded byte-by-byte with no reply at all, forever, since we
        # have no way to distinguish "still booting" from "already past
        # calibration and ignoring this on purpose" from the reply alone.
        # Retrying forever here would hang the entire link permanently in
        # that case. Give up after a handful of attempts and proceed to the
        # normal control loop anyway - real control packets always start
        # with a genuine '$', so the firmware's resync recovers on its own
        # once those start flowing, whether or not calibration is confirmed.
        MAX_CALIBRATION_ATTEMPTS = 5
        for attempt in range(1, MAX_CALIBRATION_ATTEMPTS + 1):
            log.info("Sending compass calibration to the Pico and waiting for the echo "
                      "(attempt %d)...", attempt)
            self._serial.reset_input_buffer()
            # A Pico that has only just rebooted sleeps ~2s before it starts
            # reading its USB CDC endpoint, so this 100-byte write can hit
            # the write_timeout set on the port. That is a normal race, not
            # a failure - without this the exception escapes run() and takes
            # the entire service down, which is exactly what happened the
            # first time the calibration was pushed after a picotool reboot.
            try:
                self._serial.write(payload)
            except self._serial_timeout_exc:
                log.info("Attempt %d: Pico not reading yet (still booting) - retrying...",
                         attempt)
                await asyncio.sleep(1.0)
                continue

            deadline = time.monotonic() + 2.0
            buf = b""
            while time.monotonic() < deadline and len(buf) < 100:
                chunk = self._serial.read(100 - len(buf))
                if chunk:
                    buf += chunk
                else:
                    await asyncio.sleep(0.01)

            if buf == payload:
                log.info("Pico echoed the calibration back correctly - link confirmed.")
                self._serial.reset_input_buffer()
                return
            # Second place the telemetry end-marker offset is hardcoded -
            # missing this one when the packet length changes has already
            # caused the same class of silent-desync bug twice. Check both
            # possible terminator positions, since either firmware may be
            # on the Pico.
            elif len(buf) == 100 and buf[0:1] == b"$" and (
                    buf[TELEM_LEN - 1:TELEM_LEN] == b"*"
                    or buf[TELEM_LEN_LEGACY - 1:TELEM_LEN_LEGACY] == b"*"):
                # If the Pico was already calibrated, it read our 100-byte
                # payload as ~7 separate 13-byte control packets and replied
                # to each with a full TELEM_LEN-byte telemetry packet - up to
                # ~7*TELEM_LEN bytes total, of which we only consumed the
                # first 100 above.
                # The rest are still sitting in the receive buffer; without
                # discarding them here, every subsequent read in the main
                # loop is permanently misaligned by however many stray bytes
                # are left, and since nothing re-syncs on '$'/'*' framing,
                # unpack_telemetry() then silently fails forever - no error,
                # just telemetry that never arrives. Flush before returning.
                log.info("Pico replied with normal telemetry instead of echoing calibration - "
                          "it was already past its once-per-power-up calibration gate from an "
                          "earlier run. The link is fine.")
                self._serial.reset_input_buffer()
                return
            else:
                log.warning("Attempt %d: got %d bytes, did not match (Pico may still be "
                            "booting, or not power-cycled since this service started) - "
                            "retrying...", attempt, len(buf))
                await asyncio.sleep(1.0)

        log.warning("Compass calibration handshake never confirmed after %d attempts - "
                    "proceeding anyway. If the Pico is already running (LED slow-blinking), "
                    "this is expected: real control packets resync on their own. If it's "
                    "still on solid LED waiting for calibration, it needs a power cycle.",
                    MAX_CALIBRATION_ATTEMPTS)
        self._serial.reset_input_buffer()

    async def _read_telemetry_with_timeout(self) -> dict | None:
        # Two packet lengths are possible depending on whether the Pico has
        # been reflashed with the level-capture firmware (63 bytes) or not
        # (59). Read towards the longer one and accept whichever terminator
        # actually lands, so a forgotten reflash degrades to "no level
        # panel" instead of silently killing the whole link. On the 63-byte
        # firmware byte 58 is the high byte of a zero-point clamped to
        # +/-15 degrees, so it is always 0x00 or 0xFF and can never be
        # mistaken for the legacy '*' terminator.
        deadline = time.monotonic() + self.REPLY_TIMEOUT_S
        buf = bytearray()
        while time.monotonic() < deadline and len(buf) < TELEM_LEN:
            try:
                chunk = self._serial.read(TELEM_LEN - len(buf))
            except self._serial_error_exc as exc:
                # Same dead handle as the write side - the device went
                # away. Hand back "no telemetry" and let the loop's write
                # path do the reopening, so there is exactly one place
                # that reopens the port.
                log.warning("Pico serial read failed (%s)", exc)
                self._connected = False
                return None
            if chunk:
                buf += chunk
            elif any(len(buf) == n and buf[n - 1] == END_BYTE for n in TELEM_LENS):
                break  # shorter firmware layout - nothing more is coming
            else:
                await asyncio.sleep(0.002)

        # Longest first: a 65-byte packet must never be mistaken for a
        # shorter one that happens to have '*' at the shorter terminator
        # position.
        for n in TELEM_LENS:
            if len(buf) >= n and buf[n - 1] == END_BYTE:
                return unpack_telemetry(bytes(buf[:n]))
        return None

    async def _broadcast(self, telemetry: dict):
        if not self._clients:
            return
        msg = json.dumps({"type": "telemetry", "data": telemetry})
        dead = []
        for ws in self._clients:
            try:
                # Bounded: a phone that walks out of WiFi range or sleeps
                # leaves a half-open socket whose send buffer never drains,
                # and a bare await here would hang the link loop - and with
                # it the whole server - on a client that is already gone.
                await asyncio.wait_for(ws.send_str(msg), timeout=1.0)
            except Exception:
                # Any failure sending to one client (not just a clean
                # ConnectionResetError - an abrupt tab close can raise other
                # exception types depending on OS/browser) must never be
                # allowed to propagate out of here: this is called from the
                # main run() loop, and an uncaught exception there kills the
                # entire Pico link silently - the rest of the web server
                # keeps working, but nothing gets sent to the Pico ever
                # again, with no error logged anywhere. Drop the bad client
                # and keep the link alive for everyone else.
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)


# ---------------------------------------------------------------------------
# Camera - MJPEG stream via picamera2 (Pi 5 uses libcamera, not the legacy
# raspicam/MMAL stack the old Pi Zero yct.cpp depended on).
# ---------------------------------------------------------------------------

class CameraStream:
    def __init__(self):
        self._picam2 = None
        self._available = False

    def start(self):
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import MJPEGEncoder, Quality
            from picamera2.outputs import FileOutput
        except ImportError:
            log.warning("picamera2 not importable - video stream will be unavailable. "
                        "On the Pi 5: sudo apt install -y python3-picamera2")
            return

        # Anything beyond a missing import - no camera module physically
        # connected (Picamera2() raises IndexError probing an empty camera
        # list), a camera that's busy, etc. - should degrade the same way:
        # video unavailable, rest of the dashboard (flight control) unaffected.
        # This is not hypothetical - it's exactly what happened running this
        # against real Pi 5 hardware before the camera was reconnected.
        try:
            self._output = _MJPEGOutput()
            self._picam2 = Picamera2()
            config = self._picam2.create_video_configuration(main={"size": (1280, 720)})
            self._picam2.configure(config)
            self._apply_motion_controls()
            self._picam2.start_recording(MJPEGEncoder(), FileOutput(self._output), Quality.MEDIUM)
            self._available = True
            log.info("Camera streaming started (1280x720 MJPEG).")
        except Exception:
            log.warning("Camera present in software but failed to start (no module "
                        "connected?) - video stream will be unavailable.", exc_info=True)
            self._picam2 = None
            self._available = False

    #: Longest exposure the auto-exposure algorithm may choose, microseconds.
    #: 2500us = 1/400s, the usual guidance for freezing a moving subject.
    #:
    #: This is the single most valuable camera setting for the detector, and
    #: it is worth being clear why. A shaky picture and a BLURRED picture are
    #: different problems: stabilisation moves a frame around, but motion
    #: blur is baked into the frame while the shutter is open and nothing
    #: downstream can undo it. Blur is also what actually degrades detection
    #: accuracy - a neural network does not care that the horizon wobbles
    #: between frames, it cares that the person is a smear.
    #:
    #: The cost is light sensitivity: a shorter shutter means a darker image,
    #: and the camera compensates with gain (noise). In daylight over water -
    #: this drone's entire use case - there is light to spare.
    AE_MAX_SHUTTER_US = 2500

    def _apply_motion_controls(self):
        """Bias the camera towards freezing motion rather than collecting light.

        Applied defensively: the exact control names vary between libcamera
        versions and sensors, so each is tried on its own and a rejection is
        logged rather than being allowed to stop the camera starting. A
        camera with default exposure is much better than no camera.
        """
        wanted = [
            # 1 = "short" exposure mode, the one meant for sport/motion.
            ("AeExposureMode", 1),
            ("AeMaxShutter", self.AE_MAX_SHUTTER_US),
        ]
        for name, value in wanted:
            try:
                self._picam2.set_controls({name: value})
                log.info("Camera control %s=%s applied.", name, value)
            except Exception as e:
                log.info("Camera control %s not supported here (%s) - skipping.",
                         name, type(e).__name__)

    @property
    def available(self) -> bool:
        return self._available

    async def frames(self):
        while True:
            frame = await self._output.next_frame()
            yield frame


class _MJPEGOutput(io.BufferedIOBase):
    """Bridges picamera2's synchronous FileOutput.write() callback into
    something an asyncio consumer can await, using the standard
    condition-variable handoff pattern from picamera2's own examples.

    Must subclass io.BufferedIOBase - newer picamera2 versions check this
    with isinstance() in FileOutput and raise RuntimeError otherwise (this
    is not hypothetical, it's exactly what happened running this against
    real Pi 5 hardware)."""

    def __init__(self):
        super().__init__()
        self._frame = None
        self._condition = asyncio.Condition()
        self._loop = asyncio.get_event_loop()

    def writable(self) -> bool:
        return True

    def write(self, buf):
        # Called from picamera2's own encoder thread, not the event loop -
        # hop back onto the loop to safely notify async waiters.
        asyncio.run_coroutine_threadsafe(self._set_frame(buf), self._loop)

    async def _set_frame(self, buf):
        async with self._condition:
            self._frame = buf
            self._condition.notify_all()

    async def next_frame(self) -> bytes:
        async with self._condition:
            await self._condition.wait()
            return self._frame


# ---------------------------------------------------------------------------
# HTTP / WebSocket routes.
# ---------------------------------------------------------------------------

def _webapp_index() -> Path | None:
    """Path to the built React dashboard's entry point, or None if it
    hasn't been deployed."""
    candidate = WEBAPP_DIR / "index.html"
    return candidate if candidate.is_file() else None


async def index(request):
    """The SeaYou dashboard at '/', falling back to the pilot dashboard.

    Serving the React app from the Pi itself is what lets the whole thing
    live at http://drone.local:8080 with no laptop in the loop: the page
    and the drone are then the same origin, so the app addresses /ws and
    /stream.mjpg as plain relative paths and works just as well over a
    raw IP when mDNS fails.
    """
    webapp = _webapp_index()
    return web.FileResponse(webapp if webapp else STATIC_DIR / "index.html")


async def pilot_index(request):
    """The original touch-stick pilot dashboard, always reachable at
    /pilot even once the React app owns '/'. Kept because it is the
    version with the most flight time on it - it is the fallback if the
    React dashboard misbehaves in the field."""
    return web.FileResponse(STATIC_DIR / "index.html")


async def status_handler(request):
    link: PicoLink = request.app["link"]
    camera: CameraStream = request.app["camera"]
    return web.json_response({
        **link.status,
        "camera_available": camera.available,
        "latest_control": link._latest_control,
        "latest_telemetry": link._latest_telemetry,
    })


async def shutdown_handler(request):
    # Mirrors the old Android app's power button (see the '!'+'1' command
    # in the retired tcp_uart_c.cpp) - a clean OS shutdown, not just cutting
    # power, which risks corrupting the SD card since this service is
    # actively writing logs. Respond first, then shut down shortly after so
    # the browser actually gets the confirmation before the Pi goes down.
    log.warning("Shutdown requested from the dashboard.")

    async def _do_shutdown():
        await asyncio.sleep(1.0)
        subprocess.run(["sudo", "shutdown", "-h", "now"])

    asyncio.create_task(_do_shutdown())
    return web.json_response({"shutting_down": True})


async def level_cal_handler(request):
    """Trigger a level zero-point capture without the dashboard UI, e.g.
    from the bench:  curl -X POST localhost:8080/level_cal -d '{"action":"start"}'
    GET returns the Pico's current stored zero-point and capture state."""
    link: PicoLink = request.app["link"]

    if request.method == "GET":
        telemetry = link._latest_telemetry or {}
        return web.json_response({"level": telemetry.get("level")})

    try:
        payload = await request.json()
    except Exception:
        payload = {}
    result = link.request_level_cal(payload.get("action", "start"))
    return web.json_response(result, status=200 if result["accepted"] else 409)


async def websocket_handler(request):
    link: PicoLink = request.app["link"]
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    link.register(ws)
    log.info("Dashboard client connected (%d total).", len(link._clients))

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    payload = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue
                if payload.get("type") == "control":
                    link.set_control(payload.get("data", {}), payload.get("gp"))
                elif payload.get("type") == "gp_debug":
                    link.set_gamepad_debug(payload.get("gp"))
                elif payload.get("type") == "level_cal":
                    result = link.request_level_cal(payload.get("action", "start"))
                    await ws.send_str(json.dumps({"type": "level_cal_ack", "data": result}))
            elif msg.type == WSMsgType.ERROR:
                log.warning("WebSocket error: %s", ws.exception())
    finally:
        link.unregister(ws)
        log.info("Dashboard client disconnected (%d remaining).", len(link._clients))

    return ws


async def mjpeg_handler(request):
    camera: CameraStream = request.app["camera"]
    if not camera.available:
        return web.Response(status=503, text="Camera not available")

    response = web.StreamResponse(
        status=200,
        headers={"Content-Type": "multipart/x-mixed-replace; boundary=FRAME"},
    )
    await response.prepare(request)

    try:
        async for frame in camera.frames():
            await response.write(
                b"--FRAME\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                + frame + b"\r\n"
            )
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return response


def build_app(link: PicoLink, camera: CameraStream) -> web.Application:
    app = web.Application()
    app["link"] = link
    app["camera"] = camera
    app.router.add_get("/", index)
    app.router.add_get("/pilot", pilot_index)
    app.router.add_get("/status", status_handler)
    app.router.add_post("/shutdown", shutdown_handler)
    app.router.add_get("/level_cal", level_cal_handler)
    app.router.add_post("/level_cal", level_cal_handler)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_get("/stream.mjpg", mjpeg_handler)
    app.router.add_static("/static/", STATIC_DIR)

    # The built dashboard references everything by absolute path -
    # /assets/... for Vite's hashed bundles, /fonts/... for the
    # self-hosted fonts, /img/... for the imagery - so each of those
    # directories has to be served from that exact prefix.
    #
    # Registered by walking the build rather than by listing prefixes,
    # because hardcoding them means the day someone adds a new folder
    # under public/ it silently 404s in production only. Registered only
    # when the build is present: add_static() raises on a missing
    # directory, which would take the whole flight server down at startup.
    if WEBAPP_DIR.is_dir():
        served = []
        for child in sorted(WEBAPP_DIR.iterdir()):
            if child.is_dir():
                app.router.add_static(f"/{child.name}/", child)
                served.append(child.name)
        # Loose files at the build root (favicon.svg, icons.svg) need
        # individual routes - a static route on "/" would swallow the
        # WebSocket and every API endpoint.
        for f in sorted(WEBAPP_DIR.glob("*.*")):
            if f.is_file() and f.name != "index.html":
                app.router.add_get(f"/{f.name}", _file_route(f))
                served.append(f.name)
        if served:
            log.info("Serving the SeaYou React dashboard from %s (%s); "
                     "pilot dashboard moved to /pilot",
                     WEBAPP_DIR, ", ".join(served))
    if not _webapp_index():
        log.info("No React build at %s - serving the pilot dashboard at / as usual.",
                 WEBAPP_DIR)
    return app


def _file_route(path: Path):
    """Handler serving one fixed file. Bound via a factory so each route
    captures its own path rather than sharing the loop variable."""
    async def handler(request):
        return web.FileResponse(path)
    return handler


async def main_async(args):
    link = PicoLink(args.serial, args.baud, args.compass_cal, args.log_dir)
    camera = CameraStream()
    camera.start()

    app = build_app(link, camera)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.port)
    await site.start()
    log.info("Dashboard serving on http://0.0.0.0:%d", args.port)

    await link.run()  # runs forever


def main():
    parser = argparse.ArgumentParser(description="SeaYou drone dashboard server")
    parser.add_argument("--serial", default="/dev/ttyACM0",
                        help="Serial device for the Pico link. The Pico now talks over its "
                             "USB CDC port (plugged into a Pi5 USB-A port) rather than the "
                             "GP16/17 UART pins - it should enumerate as /dev/ttyACM0. Check "
                             "with 'ls /dev/ttyACM*' if it's not found at that path.")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--compass-cal", default="/home/pi/compass_cal.txt")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--log-dir", default="/home/pi/flightlogs",
                        help="Directory for per-run flight CSV logs. One file per "
                             "service start; flights within it are marked by the "
                             "flight_id column.")
    args = parser.parse_args()

    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
