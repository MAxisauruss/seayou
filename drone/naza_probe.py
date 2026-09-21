"""Probe a DJI NAZA / Phantom GPS module on the Pi's UART.

    ssh pi@drone.local 'cd /home/pi/dashboard && python3 naza_probe.py'

DJI's GPS does not speak NMEA. It sends a proprietary binary protocol at
115200 baud, which is why the handover ruled this module out - but that
verdict was about a fixed-offset NMEA parser in C on the Pico. On the Pi, in
Python, decoding it is ordinary work.

PROTOCOL (from the community NazaDecoder implementations)

    0x55 0xAA  <id>  <len>  <payload[len]>  <cs1> <cs2>

    id 0x10  GPS,          58 byte payload
    id 0x20  magnetometer,  6 byte payload
    id 0x30  module version, 12 byte payload

    cs1 = sum of id, len and every payload byte   (mod 256)
    cs2 = running sum of cs1 after each of those  (mod 256)

The GPS payload is obfuscated: every byte is XORed with a mask that travels
in the message itself, at offset 0x37. The mask byte and the two sequence
bytes after it are not XORed.

    offset  field
    0x04    longitude,  int32 little-endian, / 1e7 degrees
    0x08    latitude,   int32 little-endian, / 1e7 degrees
    0x0C    altitude,   int32 little-endian, / 1e3 metres
    0x1C    north velocity, int32, / 100 cm/s
    0x20    east velocity,  int32, / 100 cm/s
    0x30    satellites, uint8
    0x32    fix type,   uint8   (2 = 2D, 3 = 3D)

DELIBERATELY EMPIRICAL. It reports what it actually sees before it tries to
interpret anything: raw byte counts, then framing, then a hex dump of a real
GPS message, and only then the decoded values with a sanity check. Three
times today a confident interpretation of a measurement turned out to be
wrong, so this prints the evidence alongside the conclusion.
"""

import struct
import sys
import time

import serial

PORT = "/dev/ttyAMA0"
BAUD = 115200
LISTEN_S = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0

HDR = b"\x55\xaa"
MSG_GPS, MSG_MAG, MSG_VER = 0x10, 0x20, 0x30
SIZES = {MSG_GPS: 0x3A, MSG_MAG: 0x06, MSG_VER: 0x0C}
NAMES = {MSG_GPS: "GPS", MSG_MAG: "magnetometer", MSG_VER: "module version"}

POS_LO, POS_LA, POS_AL = 0x04, 0x08, 0x0C
POS_NV, POS_EV = 0x1C, 0x20
POS_NS, POS_FT, POS_XM = 0x30, 0x32, 0x37


def hexdump(b, indent="    "):
    out = []
    for off in range(0, len(b), 16):
        chunk = b[off:off + 16]
        hexs = " ".join("%02X" % c for c in chunk)
        txt = "".join(chr(c) if 32 <= c < 127 else "." for c in chunk)
        out.append("%s%04X  %-47s |%s|" % (indent, off, hexs, txt))
    return "\n".join(out)


def checksum_ok(mid, length, payload, cs1, cs2):
    a = b = 0
    for byte in bytes([mid, length]) + payload:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return a == cs1 and b == cs2


def deobfuscate(payload):
    """XOR every byte with the in-band mask, leaving mask and sequence alone."""
    mask = payload[POS_XM]
    out = bytearray(payload)
    for i in range(len(out)):
        if i not in (POS_XM, POS_XM + 1, POS_XM + 2):
            out[i] ^= mask
    return bytes(out), mask


def decode_gps(payload):
    d, mask = deobfuscate(payload)
    lon = struct.unpack_from("<i", d, POS_LO)[0] / 1e7
    lat = struct.unpack_from("<i", d, POS_LA)[0] / 1e7
    alt = struct.unpack_from("<i", d, POS_AL)[0] / 1e3
    nv = struct.unpack_from("<i", d, POS_NV)[0] / 100.0
    ev = struct.unpack_from("<i", d, POS_EV)[0] / 100.0
    sats = d[POS_NS]
    fix = d[POS_FT]
    return {
        "lat": round(lat, 7), "lon": round(lon, 7), "alt_m": round(alt, 2),
        "sats": sats, "fix": fix,
        "speed_ms": round((nv * nv + ev * ev) ** 0.5, 2),
        "mask": mask,
    }


def plausible(g):
    """A decode that produces nonsense is a decode that is wrong."""
    return (-90 <= g["lat"] <= 90 and -180 <= g["lon"] <= 180
            and 0 <= g["sats"] <= 64 and -500 <= g["alt_m"] <= 20000)


def main():
    print("Listening on %s at %d baud for %.0f s ...\n" % (PORT, BAUD, LISTEN_S))
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.reset_input_buffer()

    raw = bytearray()
    end = time.time() + LISTEN_S
    while time.time() < end:
        raw += ser.read(1024)
    ser.close()

    print("Raw bytes received: %d" % len(raw))
    if not raw:
        print("\nNOTHING AT ALL. Either the module is not transmitting, or the")
        print("yellow TX wire is not on pin 10, or it is not powered.")
        print("Check the LED, then meter the yellow wire against GND.")
        return 1

    print("First 128 bytes as they arrived:")
    print(hexdump(bytes(raw[:128])))

    n_hdr = raw.count(HDR)
    print("\n0x55 0xAA headers found: %d" % n_hdr)
    if n_hdr == 0:
        printable = sum(1 for c in raw if 32 <= c < 127)
        print("\nBytes ARE arriving but with no NAZA framing.")
        print("  printable characters: %d of %d" % (printable, len(raw)))
        if b"$" in raw:
            print("  '$' present - this may be NMEA after all. Try naza_probe")
            print("  at other baud rates, or read it as NMEA.")
        else:
            print("  Not NMEA either. Wrong baud, or a protocol neither of us")
            print("  expected. The hex dump above is the evidence.")
        return 1

    # ---- walk the frames
    counts = {}
    bad_cs = 0
    first_gps = None
    decoded = []
    i = 0
    while i < len(raw) - 4:
        if raw[i:i + 2] != HDR:
            i += 1
            continue
        mid, length = raw[i + 2], raw[i + 3]
        if length > 0x3A or i + 4 + length + 2 > len(raw):
            i += 1
            continue
        payload = bytes(raw[i + 4:i + 4 + length])
        cs1, cs2 = raw[i + 4 + length], raw[i + 5 + length]
        if not checksum_ok(mid, length, payload, cs1, cs2):
            bad_cs += 1
            i += 1
            continue
        counts[mid] = counts.get(mid, 0) + 1
        if mid == MSG_GPS and length == SIZES[MSG_GPS]:
            if first_gps is None:
                first_gps = payload
            decoded.append(decode_gps(payload))
        i += 4 + length + 2

    print("Messages with a VALID checksum:")
    for mid, n in sorted(counts.items()):
        print("    id 0x%02X  %-16s %d" % (mid, NAMES.get(mid, "unknown"), n))
    if bad_cs:
        print("    (%d frames failed the checksum - normal at the edges of a capture)" % bad_cs)

    if not decoded:
        print("\nFraming is good but no complete GPS message was decoded.")
        print("The module is ALIVE and talking NAZA - that alone settles it.")
        return 0

    print("\nOne raw GPS payload, before de-obfuscation:")
    print(hexdump(first_gps))

    g = decoded[-1]
    print("\nDecoded (latest of %d):" % len(decoded))
    print("    fix type   : %d  (0/1 = none, 2 = 2D, 3 = 3D)" % g["fix"])
    print("    satellites : %d" % g["sats"])
    print("    latitude   : %.7f" % g["lat"])
    print("    longitude  : %.7f" % g["lon"])
    print("    altitude   : %.2f m" % g["alt_m"])
    print("    speed      : %.2f m/s" % g["speed_ms"])
    print("    xor mask   : 0x%02X" % g["mask"])

    if plausible(g):
        print("\nValues are in range - the decode is working.")
        if g["fix"] < 2:
            print("No satellite fix yet, which is expected indoors. Satellites")
            print("climbing is the thing to watch; take it outside for a fix.")
    else:
        print("\nValues are OUT OF RANGE, so the decode is wrong even though the")
        print("framing and checksums are good. The de-obfuscation or the offsets")
        print("need adjusting against the hex dump above.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
