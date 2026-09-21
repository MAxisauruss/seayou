"""INA219 battery monitor on the Pi's own I2C bus (i2c-1, GPIO2 / GPIO3).

Deliberately on the Pi and not the Pico. The Pico's I2C bus already carries
the IMU, magnetometer and barometer and is known to be marginal - i2c_faults
is a telemetry field precisely because that bus glitches. Hanging a fourth
device off it to watch the battery would put the sensors that keep the
aircraft upright at risk for the sake of a number that is not flight
critical.

Sampling runs in its own thread at 2 Hz. The 30 Hz flight loop never performs
an I2C transaction and never blocks on one - it reads whatever the last
completed sample was, or None. A hung bus costs a stale battery reading and
nothing else.

Reporting a wrong battery voltage is worse than reporting none. Two failure
modes are therefore detected and reported as faults rather than as readings:

  * open_shunt  - the shunt amplifier is railed at full scale while the bus
                  input reads 0 V. That is VIN+ driven with VIN- floating:
                  the load side of the shunt is not connected. Without this
                  check the dashboard would show a confident 0.0 V, which
                  looks exactly like a flat pack.
  * overflow    - the INA219's own math-overflow flag.
"""

import logging
import threading
import time

log = logging.getLogger(__name__)

I2C_BUS = 1
I2C_ADDR = 0x40

_REG_CONFIG = 0x00
_REG_SHUNT = 0x01
_REG_BUS = 0x02

# BRNG=32V, PGA=/8 (+/-320 mV), bus and shunt ADCs both 128-sample averaged
# (~68 ms per conversion), continuous shunt-and-bus mode.
_CONFIG = (1 << 13) | (3 << 11) | (0x0B << 7) | (0x0B << 3) | 0x07

_SHUNT_LSB_MV = 0.01        # 10 uV per LSB
_BUS_LSB_V = 0.004          # 4 mV per LSB, value is in bits 15:3
_PGA_FULL_SCALE_MV = 320.0

# The resistor fitted to the breakout. Generic GY-219 and Adafruit INA219
# boards both ship 0.1 ohm. Only affects the current figure, never voltage.
SHUNT_OHMS = 0.1

SAMPLE_PERIOD_S = 0.5

# VIN+ and VIN- are jumpered together onto battery positive, so the shunt sits
# in no current path at all. That is deliberate - this 0.1 ohm part could never
# carry motor current (10 W at 10 A) - but it means the shunt reading is just
# noise around zero and any current derived from it is meaningless. Measured
# after the rewire: 0.02 mV typical with an occasional 80 mV spike from bus
# contention, which would have been reported as a phantom 0.8 A.
#
# Set this True only if the module is ever rewired with VIN- feeding a real
# load, which needs a shunt sized for that load.
CURRENT_SENSE_WIRED = False


class BatteryMonitor:
    """Background INA219 reader. Safe to construct when no sensor is fitted."""

    def __init__(self, bus_num: int = I2C_BUS, addr: int = I2C_ADDR,
                 shunt_ohms: float = SHUNT_OHMS, cells: int | None = None):
        self._bus_num = bus_num
        self._addr = addr
        self._shunt_ohms = shunt_ohms
        self._cells = cells
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._present = False
        self._fault_count = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._smbus = None

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        try:
            import smbus2
        except ImportError:
            log.warning("smbus2 not installed - battery monitor disabled.")
            return
        try:
            self._smbus = smbus2.SMBus(self._bus_num)
            self._write(_REG_CONFIG, _CONFIG)
            time.sleep(0.1)
            # A present INA219 reads back the config it was just given. A
            # missing one raises OSError on the write above instead.
            if self._read(_REG_CONFIG) != _CONFIG:
                log.warning("INA219 at 0x%02X did not accept its configuration "
                            "- battery monitor disabled.", self._addr)
                self._smbus = None
                return
        except Exception as exc:
            log.warning("No INA219 on i2c-%d at 0x%02X (%s) - battery "
                        "monitor disabled.", self._bus_num, self._addr, exc)
            self._smbus = None
            return

        self._present = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="battery")
        self._thread.start()
        log.info("INA219 battery monitor started on i2c-%d addr 0x%02X "
                 "(shunt %.3f ohm).", self._bus_num, self._addr,
                 self._shunt_ohms)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    @property
    def present(self) -> bool:
        return self._present

    # -- sampling ----------------------------------------------------------
    def _write(self, reg: int, val: int):
        self._smbus.write_i2c_block_data(
            self._addr, reg, [(val >> 8) & 0xFF, val & 0xFF])

    def _read(self, reg: int) -> int:
        d = self._smbus.read_i2c_block_data(self._addr, reg, 2)
        return (d[0] << 8) | d[1]

    def _run(self):
        while not self._stop.wait(SAMPLE_PERIOD_S):
            try:
                sample = self._sample()
            except Exception as exc:
                self._fault_count += 1
                # Logged once per 20 faults: a disconnected sensor would
                # otherwise fill the log at 2 Hz forever.
                if self._fault_count % 20 == 1:
                    log.warning("INA219 read failed (%s), fault #%d",
                                exc, self._fault_count)
                sample = {"fault": "i2c_error", "i2c_faults": self._fault_count}
            with self._lock:
                self._latest = sample

    def _sample(self) -> dict:
        shunt_raw = self._read(_REG_SHUNT)
        if shunt_raw > 32767:
            shunt_raw -= 65536
        bus_raw = self._read(_REG_BUS)

        shunt_mv = shunt_raw * _SHUNT_LSB_MV
        bus_v = (bus_raw >> 3) * _BUS_LSB_V
        overflow = bool(bus_raw & 0x01)
        railed = abs(shunt_mv) >= _PGA_FULL_SCALE_MV * 0.999

        out: dict = {
            "voltage_v": round(bus_v, 3),
            "shunt_mv": round(shunt_mv, 3),
            "i2c_faults": self._fault_count,
        }

        # The INA219 measures BUS VOLTAGE AT THE VIN- PIN, relative to GND.
        # VIN+ is only ever one side of the current shunt. So where the pack's
        # positive lead lands decides whether a voltage can be read at all,
        # and the four cases below are distinguishable from the two registers:
        #
        #   bus high, shunt sane    normal - volts and amps both good
        #   bus high, shunt railed  volts good; current meaningless (see below)
        #   bus ~0,   shunt railed  the pack is ACROSS VIN+ and VIN- - i.e.
        #                           wired like a voltmeter's two probes. That
        #                           is the 0.1 ohm shunt straight across the
        #                           battery, which is a ~120 A fault current
        #                           and will have opened the resistor. The
        #                           chip can never read volts like this,
        #                           because bus voltage is measured AT VIN-,
        #                           and here VIN- IS battery negative.
        #
        #                           DO NOT advise jumpering VIN+ to VIN- in
        #                           this state - with battery- still on VIN-
        #                           that is a dead short across the pack.
        #                           battery- has to move to the GND pin
        #                           FIRST.
        #   bus ~0,   shunt ~0      nothing connected at all
        if bus_v < 1.0:
            if railed:
                out["fault"] = "pack_across_shunt"
                out["hint"] = "battery- must move to GND before anything else"
            else:
                out["fault"] = "no_pack"
                out["hint"] = "no pack voltage on VIN-"
            out["voltage_v"] = None
            out["current_a"] = None
            return out

        if overflow:
            out["fault"] = "overflow"
            out["current_a"] = None
            return out

        # The voltage is real and worth showing; the current is not, so it is
        # withheld rather than reported as a plausible-looking wrong number -
        # whether because the shunt is railed or because it is simply not in a
        # current path at all.
        if railed or not CURRENT_SENSE_WIRED:
            out["current_a"] = None
            out["note"] = "voltage only - shunt not in a current path"
        else:
            out["current_a"] = round(shunt_mv / 1000.0 / self._shunt_ohms, 3)
            out["power_w"] = round(bus_v * out["current_a"], 2)

        cells = self._cells or self._guess_cells(bus_v)
        if cells:
            out["cells"] = cells
            out["cell_v"] = round(bus_v / cells, 3)
        return out

    @staticmethod
    def _guess_cells(v: float) -> int | None:
        """Cell count from pack voltage, for the per-cell figure only.

        A LiPo cell runs 3.0-4.2 V. Ranges for 1S-6S do not overlap above
        3.0 V/cell, so this is unambiguous for any pack that is not already
        deeply over-discharged - and returns None rather than guessing when
        the voltage falls in no valid band.
        """
        if v < 2.9:
            return None
        for n in range(1, 7):
            if 3.0 * n <= v <= 4.25 * n:
                return n
        return None

    # -- consumer ----------------------------------------------------------
    def read(self) -> dict | None:
        """Last completed sample, or None if no sensor / none taken yet."""
        with self._lock:
            return dict(self._latest) if self._latest else None
