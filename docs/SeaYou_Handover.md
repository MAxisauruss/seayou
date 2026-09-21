# SeaYou Drone — On-Site Handover

2026-09-17, updated 2026-09-20 (GPS firmware, INA219 battery monitor,
power-supply load test)

Offline copy of the handover doc:
https://claude.ai/code/artifact/3a17349d-bd46-4d43-a583-ff66893f3e09

---

## Start here

Two jobs, in this order.

**1. Power the drone's 5 V rail from the flight battery.**

The GPS is not broken and the firmware is not the problem. Per
`V4_Wiring_Diagram.png`, the GPS module's **red VCC wire goes to the "ESC's
5V power supply" rail** - the same rail that feeds the Pico's VSYS on physical
pin 39 and the I2C sensors. It is NOT on the Pico's 3V3 output.

**That rail is currently being fed only by USB.** The Pico measures its own
VSYS on ADC3, and it reads **4.79-4.83 V** with VBUS present - which is USB
5 V minus the Pico's on-board Schottky drop. A live ESC BEC would hold it at
5.0 V or above. So the ESCs are not powering anything.

That is one root cause for both open faults: a GPS on an unpowered rail says
nothing, and an INA219 whose pack side is not connected reads nothing.

Connect the flight battery to the ESC power distribution, then watch the
**5V RAIL** figure in the flight bar. When it goes green (`ESC BEC live`),
re-check the GPS - `fix` moves off `0` the moment one valid sentence arrives.

**If the rail is live and the GPS is still silent**, then put a multimeter on
the module.

**1b. If you still need to probe by hand.** The firmware side is done — see
*GPS: resolved in firmware, 20 Sep* below. The Pico now reports that it is
receiving **zero bytes** from the receiver at every baud rate it tries, so
what is left is electrical, not code.

1. **Ground first.** Continuity between the GPS module's `GND` pin and any
   Pico `GND` (physical pin 38 is convenient). This is the top suspect: both
   GP4 and GP5 currently sit parked at a hard 3.3 V and never move, which is
   what a missing ground reference looks like.
2. **Power.** Measure between the GPS module's `VCC` and `GND` pins. You
   should see 3.3 V (or 5 V if it is a 5 V module). Nothing there is the
   whole answer.
3. **Continuity.** The GPS module's **TX** must reach the Pico's **GP5 —
   physical pin 7**. That is the Pico's UART1 receive pin. If TX and RX are
   swapped, this is exactly what you see: silence.
   (The Pico's GP4, physical pin 6, is UART1 TX → GPS RX. Only needed for
   configuring the module, which the firmware no longer does.)
4. **Check nothing landed on VCC.** Both Pico pins being held hard at 3.3 V
   would also be explained by a data wire sitting on the module's supply pin.
5. **Watch the dashboard as you work.** The fix field now distinguishes
   `0` silent from `1` searching, so the moment a single valid sentence
   arrives it changes — you do not need a fix, or even a window, to know you
   have fixed the wiring. A GPS talks the instant it has power.

A USB-serial adapter at 9600 baud is still the fallback if the multimeter
says everything is connected: if the module is silent there too, it is dead.

**2. Set level on a genuinely flat surface, then measure the drift.** The
stored pitch offset is currently suspect.

1. Drone on a flat surface, throttle at zero.
2. Press **SET LEVEL** in the flight bar. Hands off for three seconds.
3. Check the attitude readout — it should sit near **0° / 0°**. If it still
   reads several degrees off right after a successful capture, the sensor
   board is physically mis-mounted and no calibration will fix it.
4. Then tether it, **TAKE OFF to 2 m**, and leave the sticks alone for ten
   seconds.

That last step is the measurement that has been missing all along. Every log
so far has zero hands-off airborne samples, so nobody can yet say whether the
drift is real.

---

## Addresses and where the credentials live

Secrets are deliberately **not written here**. Each one lives in a file on the
ground-station machine.

| What | Where to find it |
| --- | --- |
| Ground station login | Username `admin`. Password set 16 Sep. Lost it? Delete `users.json` beside the exe and relaunch — a new one is printed once, at the bottom of the window. |
| Drone token | `seayou_station.json`, beside the exe. Stable across restarts, which is why the Pi stays configured. |
| Viewer token (read-only mirror) | Same file, `viewer_token`. Printed at startup too. |
| Pi login | `pi@drone.local` over SSH, key-based — no password needed from this machine. |

**Move `users.json` and `seayou_station.json` with the exe if you ever relocate
it.** Without them you get a new password and the Pi silently stops connecting.

| Machine | Address |
| --- | --- |
| Laptop | `DESKTOP-CO03PS0` — resolves over mDNS from the Pi. House WiFi `192.168.0.162`; hosting its own hotspot `192.168.137.1`. |
| Pi | `drone.local`, `192.168.0.128` on wlan0 |
| Ground station | http://localhost:8090 |
| Public mirror | seayou-indol.vercel.app — read-only, needs `?relay=` and the viewer token |

The Pi's `seayou-agent` service tries **three** ground-station addresses in
turn — `DESKTOP-CO03PS0.local`, `192.168.137.1`, `192.168.0.162` — so it finds
the laptop on whichever network you are both on without anyone editing a unit
file at the field.

---

## Running it

Double-click **`groundstation/dist/SeaYouGroundStation.exe`**. That is the
whole procedure. It starts the ground station, opens the dashboard in a
browser, starts the public tunnel if `cloudflared` is installed, and prints one
block with everything in it: the dashboard link, the exact `drone_agent.py`
command with the laptop's address already filled in, and the mirror link.

```
Pico --USB--> Pi 5 (drone_agent.py) --WS out, hotspot--> Laptop (ground station)
                                                            |
                                              +-------------+-------------+
                                              |                           |
                                        You fly here            cloudflared -> Vercel
                                                                (watches only)
```

The drone dials **out**, so it never needs a reachable address. The mirror is a
side branch: the aircraft never touches it, so it cannot add latency to flying.

### If the dashboard looks out of date

**Close the exe window and reopen it.** The dashboard is bundled *inside* the
executable, so refreshing the browser re-fetches from the same running exe and
shows the same old build. This has caused confusion twice.

### On the Pi

`seayou-agent.service` is installed and starts at boot. It replaced the old
`seayou-dashboard` — both want `/dev/pico` and only one can run.

```bash
systemctl is-active seayou-agent
tail -f /home/pi/dashboard/agent.log
```

Two files live on the Pi at `/home/pi/dashboard/`: `mission.py` and
`drone_agent.py`. Rebuild the exe with `python groundstation/build_exe.py`
after any dashboard change — and close the running exe first, or the build
fails with a permission error.

---

## The GPS finding

**The firmware may be why the GPS has never worked.** It is not necessarily a
dead receiver.

In `fc_quad_gt_cc.cpp`, `GPS_decode()` requires the `GN` talker ID:

```c
if (gps_raw_buff[i+2] == 'N' && gps_raw_buff[i+4] == 'G' && ...   // $GNGGA
if (gps_raw_buff[i+2] == 'N' && gps_raw_buff[i+4] == 'S' && ...   // $GNGSA
```

`GN` means **multi-constellation** (GPS + GLONASS/Galileo/BeiDou). A
**GPS-only receiver emits `$GPGGA` / `$GPGSA`** — that byte is `'P'`, the check
fails, and nothing is ever parsed. The result looks exactly like a broken
module: 0 satellites, no fix, forever, even under a clear sky.

The firmware also assumes a lot else:

- It sends a raw **u-blox UBX** message (`0xB5 0x62 0x06 0x00 …`) to
  reconfigure the receiver — so it must be genuine u-blox.
- It switches the GPS to **921600 baud**, which the NEO-6 generation cannot
  reach.
- It extracts fields by **fixed character offsets** and assumes HDOP is exactly
  4 characters. Real receivers emit `1.2` or `0.95` depending on conditions, so
  even a correct module may parse intermittently.

### Modules checked

| Module | Verdict |
| --- | --- |
| DJI Phantom 1 GPS (NEO-6Q) | **No.** DJI modules output a proprietary binary protocol, not NMEA — that is why the `NazaDecoder` library exists. GPS-only chip, and it will not answer UBX. Fails on all three counts. |
| robotics.org.za `NEO-M8N-MOD` | **No, despite the part number.** The actual product is a *Mini GPS Module (ATGM336H)* — a CASIC chip, not u-blox. It cannot be configured by UBX, so it stays at 9600 while the Pico listens at 921600. The "M8N" in the code is search bait. |
| Genuine u-blox NEO-M8N | **Yes** — multi-GNSS, answers UBX, does 921600. |
| NEO-6M / NEO-7M | No — GPS-only, wrong talker ID. |

A real u-blox M8 locally is either ~R2,471 (Waveshare Pi HAT NEO-M8T) or out of
stock.

### The better option

Given that, **adapt the firmware to the modules you can actually buy** rather
than hunting one that matches firmware written around someone else's hardware.
Two small changes to `fc_quad_gt_cc.cpp`:

1. Accept `'P'` as well as `'N'` at `i+2`, so `$GPGGA` / `$GPGSA` parse.
2. Stop forcing 921600 — run the GPS at its default baud.

That would make an R198 ATGM336H, a NEO-6M, **or quite possibly the module
already fitted** work. It means reflashing the Pico, so it is a decision, not a
default. Not yet written.

---

## GPS: resolved in firmware, 20 Sep

Written, tested and flashed. The parser was rewritten and the Pico reflashed
over USB (`picotool` on the Pi — no BOOTSEL button needed, the 1200-baud
touch works). **Four** independent faults were found, not the two above.
Any one alone produces "0 satellites, no fix, forever":

1. **Talker ID.** `$GP` is now accepted alongside `$GN`, and in fact any
   talker — `$GL`, `$GA`, `$GB`.
2. **Forced baud.** `GPS_init()` no longer sends the u-blox UBX frame or
   switches to 921600. Instead the firmware **hunts**: it listens at 9600,
   38400, 115200, 57600 and 4800 in turn, rotating every 3 s until valid
   sentences appear, then stays. Nothing has to be assumed about the module.
3. **Brittle field cutting.** Parsing is comma-delimited, not fixed-offset,
   and every sentence's `*` checksum is verified before it is believed. HDOP
   of any width now parses.
4. **The hemisphere character was never stored** — this one was not known
   before and is fatal on its own. The old code packed latitude as
   `ddmm.mmmmm` with no `N`/`S`. `_nmea_to_decimal()` in `server.py`
   *requires* a trailing hemisphere and returns `None` without one, so a
   position could never have reached the dashboard **even with a perfect
   fix**. It also meant southern latitudes arrived positive — for this site,
   a 50 km error into the wrong hemisphere.

Two more latent bugs were fixed in passing: `Fix_type` could sit at its boot
value of `0`, matching neither packet branch and leaving stale coordinates in
the buffer; and `DMA0_configure()` declared a local `int dma_chan1` that
shadowed the global the IRQ handler uses — harmless only because this happens
to be the first DMA channel claimed and gets 0. Claim one more channel above
it and GPS reception would have died silently.

### What the dashboard now tells you

`fix` is no longer just "no fix":

| `fix` | Meaning |
| --- | --- |
| `0` | **Silent.** No valid NMEA reaching the Pico at all — power, wiring, or a dead module. |
| `1` | **Searching.** Receiver alive and sending valid sentences, no satellite fix yet. Watch `sat_count` climb. |
| `2` / `3` | 2D / 3D fix. |

Satellite count is now reported **without** a fix too. The firmware used to
force it to zero, which defeated `server.py`'s own handling — the one number
that tells you the receiver is alive and searching rather than dead.

While there is no fix, the three bytes HDOP would occupy carry diagnostics
instead (nothing on the Pi reads HDOP): byte 28 is the baud index currently
being tried, bytes 29–30 the raw bytes received since boot.

### Measured on the aircraft

**The GPS line is electrically static. Nothing is transmitting.**

Measured with `gps_sniffer` (below), which is the trustworthy instrument here:

- **8.68 million consecutive GPIO samples of GP5, zero transitions, line high
  100% of the time.** Repeated over many sweeps.
- **Silence at 4800, 9600, 19200, 38400, 57600, 115200 and 230400**, with the
  UART's framing/parity/break/overrun bits read explicitly, with the FIFO both
  enabled and disabled. Not one received word, good or bad.
- **Both GP5 *and* GP4 read DRIVEN HIGH** — they override the internal
  pull-up *and* pull-down, so something low-impedance is holding both at
  3.3 V, and neither ever moves.

A GPS TX line idles high but must pulse low every second. Two pins parked
hard at 3.3 V and never moving is what a **power rail** looks like, not a
data line. Prime suspects, in order: a data wire landed on VCC, or **the
module's GND is not actually shared with the Pico's** — with no ground
reference nothing can communicate and the output floats to VCC.

### A correction worth recording

The firmware's in-loop counters — `gps_raw_bytes` (DMA words) and
`gps_line_transitions` — **both read non-zero and climbing while all of the
above was true.** They were briefly believed, and they were wrong.

The DMA re-arms on completion and hands over a word whether or not a real
start bit produced it, and sampling a pin once per main loop says nothing
reliable about a signal. Both counters are now commented in the source as
hints only, and the packet comment says so too.

**The number that is trustworthy in flight telemetry is `fix` itself.** It
only leaves `'0'` when a sentence has passed its checksum, which line noise
will not do. For anything more detailed, flash `gps_sniffer`.

### gps_sniffer — the instrument

`Raspberry Pi Pico/gps_sniffer/` is a standalone diagnostic build that does
nothing but look at GP4/GP5: a pure GPIO probe (pull-up/pull-down to tell
driven from floating, plus transition counting) and a UART sweep across every
plausible baud that dumps hex, ASCII and the error bits. It is the remote
equivalent of the USB-serial adapter this handover originally asked for.

It configures no PWM at all, so the ESC pins stay high-impedance throughout.

```bash
# flash the sniffer, watch, then put the flight firmware back
ssh pi@drone.local 'sudo systemctl stop seayou-agent && sudo picotool reboot -f -u && sleep 3 && sudo picotool load -x /home/pi/firmware/gps_sniffer.uf2'
ssh pi@drone.local 'timeout 60 cat /dev/pico'
ssh pi@drone.local 'sudo picotool reboot -f -u && sleep 3 && sudo picotool load -x /home/pi/firmware/flight_controller_gpsfix.uf2 && sudo systemctl start seayou-agent'
```

If the module turns out to be the DJI Phantom unit, it speaks a proprietary
binary protocol and no NMEA parser will ever read it — the sniffer would show
real transitions and hex bytes with no `$` anywhere.

### Testing

45 host-side checks cover the parser — real `$GPGGA`/`$GNGGA` sentences,
variable field widths, checksum rejection, ring-buffer wrap-around, garbage
resilience, the baud hunt, both hemispheres, and a round-trip back through
`server.py`'s own `_nmea_to_decimal()`. They compile and run **on the Pi**
(there is no host C compiler on the laptop) against the firmware source
extracted verbatim, so they test the real code:

```bash
ssh pi@drone.local 'cd /tmp/nmea && g++ -O1 -o nmea_test nmea_test.cpp && ./nmea_test'
```

Two of those tests caught a real design bug before it ever flew: fix state
was originally taken from GSA alone, so a receiver that sends GGA but no GSA
would have parsed a perfect position and thrown it away. It now derives the
fix from both, and treats a loss reported by *either* as a loss.

### Rollback

The previous firmware is on the Pi at
`/home/pi/firmware/flight_controller_level.uf2` (byte-identical to the
12 Sep known-good snapshot):

```bash
ssh pi@drone.local 'sudo systemctl stop seayou-agent && sudo picotool reboot -f -u && sleep 3 && sudo picotool load -x /home/pi/firmware/flight_controller_level.uf2 && sudo systemctl start seayou-agent'
```

---

## Level zero-point and the drift

**SET LEVEL is in the flight bar**, next to STOP, showing the live offset.
There is a second copy in the AUTONOMOUS panel. It was missing from the React
dashboard entirely until 15 Sep — the Pico, the Pi and the ground station all
supported it, but the browser had no way to trigger it.

What it does: settles 1 s, averages 2 s, and refuses if the drone moves, if the
throttle is up, or if it is more than **15°** out. That last one means the board
is mounted wrong and no calibration will fix it.

### Where the offsets stand

| When | Roll | Pitch |
| --- | --- | --- |
| Before 16 Sep | -4.50° | 0.12° — effectively uncalibrated |
| After your capture | -6.15° | 8.43° |

**Worth re-checking.** Straight after that successful capture the aircraft
still reported **pitch +6.89°** sitting still. A fresh calibration should leave
it near zero on the surface it was calibrated on. Either it was captured on a
slope, or it has moved since. 8.43° is a large offset against a 15° limit.

### What is still unknown

Whether the drone actually drifts. **No log contains a single hands-off
airborne sample** — every flight so far has the sticks held. On 15 Sep
`ctrl_pitch` sat at 94–100 (full forward) for **100% of the 3.5 s airborne
window**.

That also means two earlier conclusions were withdrawn:

- The "pitch bias the pilot is fighting" came from averaging across ground time
  as well as flight. Airborne, the stick was simply pinned.
- The 12% motor split (m2+m3 harder) is explained by that held pitch command,
  not by a centre-of-gravity offset.

The tethered takeoff-and-hold is what settles it: guidance commands roll and
pitch to **exactly 50**, so the log finally gets airborne samples with the
sticks centred.

### Trim

**DRIFT TRIM** (±12°) sits in the settings drawer and in the flight bar. The
Pico *adds* it to your stick rather than replacing it, so you keep full
authority. It only reaches the aircraft **while TAKE CONTROL is held** — an
automatic takeoff sends neutral trim.

A 75% permanent offset was asked for and not built: that is ~22°, against a
dashboard limit of 12° and a firmware constant of 15° whose comment reads
*"beyond this, remount the board"*. Set level first, then trim, then measure —
trimming out a wrong zero-point only hides it.

---

## The 15 Sep crash — all four motors stopped at 6.75 m

**Cause: the Pi fell off the laptop hotspot.** Control went stale, the failsafe
cut the throttle, and it dropped.

From `flight_20260915_105015.csv` at t=46.959 (10:51:02):

```
t=46.198  thr 36  ctrl 26/94/50   motors 305/275/469/391   6.75 m
          <- 726 ms gap in the log ->
t=46.924  thr  0  ctrl 50/50/50   motors 232/278/514/416   6.75 m
t=46.959  thr  0  ctrl 50/50/50   motors   0/0/0/0
```

`ctrl 50/50/50` with throttle 0 **is** `SAFE_CONTROL` — the Pi's 0.5 s
control-staleness failsafe. The agent log confirms the network cause: connected
via `192.168.137.1`, then a reconnect attempt at 10:51:43 followed by `Network
is unreachable`.

What it was **not**: the `link` column stayed 1 throughout, and that column is
the **Pico serial link** — the flight controller was healthy. `i2c_faults` 0.
Not a brownout.

### Why the earlier fix did not catch it

Land-on-link-loss had been added, but it only ran when a **mission** was
active. `_select_control()` returned immediately when the mission was idle —
which is exactly what manual flight is. The pilot's sticks were replaced by
`SAFE_CONTROL` and nothing intervened.

### Fixed

The agent now tracks `_last_live_throttle` — the last throttle a *live* pilot
asked for, captured only while the link is fresh, because once stale `control`
is already SAFE and tells you nothing. If the link goes stale with that
throttle above zero, it runs `Mission.land()`: controlled descent, then a
throttle ramp near the ground instead of a cut. It falls back to the idle
failsafe only when there is no barometer.

Reproduced in the simulator — manual throttle to 7.5 m, no mission, ground
station killed outright:

```
8.59 -> 6.96 -> 4.11 -> 2.13 -> 1.18 -> 0.00 m
motors 450 -> 410 -> 380 -> 300 -> 140
```

This was the **fifth** instance in this codebase of "stop commanding =
throttle 0 = it falls". Treat that as the default hazard of any new state.

---

## Autonomy: what exists now

All of it runs **on the Pi**, inside the same 30 Hz loop that drives the Pico.
`server.py` applies the staleness failsafe *before* calling the autonomy hook
and sends whatever the hook returns — so the aircraft can fly itself with the
network completely gone.

| Feature | Needs | State |
| --- | --- | --- |
| **TAKE OFF** | Barometer only | Works. Climbs at 0.5 m/s to 1–10 m, then holds. Refuses above 1 m (already flying). |
| **LAND** | Barometer only | Works. Descends at 0.4 m/s; below 0.9 m stops trusting the baro and ramps throttle to zero over 3 s. |
| **FLY TO WAYPOINT** | GPS fix | Refused every time — no fix has ever existed. |
| **Hover trim** | Barometer | Learns the true hover throttle. Shown as **HOVER LEARNED**. |
| **Drowning detect** | Camera | No camera fitted. |

Takeoff and land need **no GPS**, which is why they are the autonomy you can
actually use today.

### Safety rules around them

- **A stick input aborts instantly.** No button to find, no confirmation.
  Verified from a climb.
- **Losing the link lands the aircraft**, in manual flight and during a
  mission. `--autonomous-on-link-loss` keeps flying a mission instead.
- **A timed-out takeoff lands**, it does not abort — aborting means throttle 0,
  and a timed-out takeoff is airborne by definition.
- **ABORT is not LAND.** Abort idles the motors; from 2 m that is a 2 m drop.
  LAND is the button for coming down on purpose.
- **Neither holds position.** Roll and pitch are held level only — wind walks
  the aircraft sideways the whole time.
- Guidance is capped: 8° tilt, 300 m range, 1–30 m height, and **85% throttle**
  ceiling.

### HOVER LEARNED — it measures the hover throttle for you

The height hold is now P+I. The old pure-proportional controller settled at a
permanent error of `(true_hover - HOVER_THROTTLE) / 6` — bench-measured: with
the simulator hovering at 60 and the code assuming 45, a 5 m takeoff parked at
exactly **2.50 m** and never arrived.

The integral trim walks the bias to where it should be, and reports it. **After
one steady hover, HOVER LEARNED is a measurement of your real hover throttle** —
the number `HOVER_THROTTLE` should be set to. Write it down when it settles.

One caveat: the integral gain is tuned against the *simulator's* vertical
response. The real aircraft's is unknown. Watch the first tethered hover for
throttle pumping; if it hunts, halve `ALT_I_GAIN`.

---

## The barometer

**It works.** Confirmed on the real aircraft: a BMP388, chip ID checked at
init, full Bosch compensation, pressure oversampling x8. Live readings 866–869
hPa at 25–28 °C, `gyro_cal_ok: true`, `i2c_faults: 0`.

**But it is coarse, and the sensor is not the reason.** The firmware sends
pressure as `(uint16)(pascals / 10)`, so the smallest change that can cross the
wire is 10 Pa. What that is worth in metres depends on where you are:

    step (m) = 843.7 / pressure_hPa

| Location | One step |
| --- | --- |
| Sea level | 0.83 m |
| **Where the drone actually is** (~1,280 m, 866 hPa) | **0.97 m** |

The dashboard computes this from live pressure rather than assuming sea level.

**Sitting completely still on the bench, the reading flickers one whole step** —
an apparent ~1 m jump with nothing moving. Expect the height hold to hunt by
about that much. That is measurement noise, not the controller misbehaving.

This limit sizes everything downstream: `ALT_ARRIVE_M` 1.0 m, `LAND_CUTOFF_M`
0.9 m. A 3 m flight has only about three distinct height values available to
it.

---

## Deploying the Vercel mirror

**Push two files** from `seayou-vercel/` into whatever repo deploys to
seayou-indol.vercel.app:

```
src/services/droneTelemetry.ts    <- NEW (the whole services folder is new)
src/pages/LiveFeeds.tsx           <- replaces the existing one
```

That is all. No config changes, no environment variables.

### Traps, both already hit

**The `services` folder is new** — it does not exist in the repo yet, so
"I can't see it there" is expected. The last deploy failed with
`TS2307: Cannot find module '../services/droneTelemetry'` because
`LiveFeeds.tsx` went up without it.

Easiest reliable route: in GitHub, **Add file -> Create new file**, and type the
path `src/services/droneTelemetry.ts` — typing the `/` creates the folders.

**Case matters.** Vercel builds on Linux: `services` lowercase,
`droneTelemetry.ts` with that exact capital T.

### Do not touch the vite config

An earlier instruction to push `vite.config.ts` was **wrong**. `vite.config.js`
shadows the `.ts` entirely — proven by setting a test base in the `.ts` and
watching the build ignore it. The leftover `base: '/my-website/'` in the `.ts`
was never active, and the live site was never at risk from it.

### What the mirror does

With **no** `?relay=` it opens no socket at all and every panel keeps its
original demo content — so deploying cannot change what a visitor to the plain
link sees. It only comes alive when opened as:

    https://seayou-indol.vercel.app/?relay=<https tunnel url>&token=<viewer token>

It is read-only by construction: that module has **no send path at all** — no
`send`, no arm, throttle or gamepad code anywhere in the tree. The ground
station enforces the same rule independently.

---

## LTE dongle — and the power warning

**Huawei E3272s-927, inspected on the laptop 15 Sep.** It enumerates as
`VID_12D1 & PID_14FE` — Huawei's "zero-CD" installer mode. It presents a
virtual CD and an empty TF-card slot, and **no network interface at all** until
something switches it. Ejecting the virtual CD did not trigger the switch.

**It is a Telkom unit.** The CD volume is labelled `Telkom Mobile` and its
`SysConfig.dat` carries `operator=C372`, Huawei's carrier code for Telkom South
Africa.

**Carrier branding is not the same as a SIM lock.** The lock is a separate
modem flag and could not be read in zero-CD mode. To settle it: switch it to
modem mode then check the web UI at `192.168.8.1`, or put a non-Telkom SIM in
and see whether it demands an unlock code.

### On the Pi it will do the same thing

```bash
sudo apt install usb-modeswitch usb-modeswitch-data
```

Its udev rules handle `12d1:14fe` automatically. After switching, these sticks
normally re-enumerate around `12d1:1506` and appear as a plain network
interface (`eth1` or `usb0`) on DHCP — no APN setup needed. Without that
package you get a CD-ROM and no internet, with nothing obvious to say why.

### Power — read before it goes on the airframe

Power was **not measured**; Windows does not expose `bMaxPower` and zero-CD mode
would not reflect the modem's real draw anyway. Planning range only: ~200–300 mA
idle, ~500–700 mA passing data, **1 A+ on transmit bursts**.

**The binding constraint is the Pi, not the stick.** The Pi 5 caps *total* USB
current at **600 mA** unless it negotiates a 5 A USB-PD supply — and a buck
converter will not negotiate. The Pico is already on that budget.

`usb_max_current_enable=1` in `/boot/firmware/config.txt` lifts it to 1.6 A, but
that only tells the firmware to *allow* it. If the buck cannot deliver, the
failure moves from the USB port to the whole Pi — and the brownout problem is
still open.

**Give the stick its own 5 V rail** (powered hub, or a Y-cable with the power
leg on a separate buck) so the Pi only sees data. Check
`vcgencmd get_throttled` first — anything but `0x0` means undervoltage already.

---

## Battery monitor — INA219, added 20 Sep

An **INA219** is fitted to the **Pi 5** GPIO header, not the Pico: pin 3
(SDA), pin 5 (SCL), at address `0x40` on `i2c-1`. Deliberately the Pi's bus —
the Pico's already carries the IMU, magnetometer and barometer and is known
to be marginal, and the battery reading is not flight critical enough to risk
the sensors that keep the aircraft upright.

`dtparam=i2c_arm=on` had to be uncommented in `/boot/firmware/config.txt`;
there was no `/dev/i2c-1` at all before, only the HDMI buses 13 and 14.

It is an INA219, not the INA226 the part number might suggest: its config
register reads back `0x399F` after reset, which is the INA219 power-on
default. An INA226 reads `0x4127` and reports manufacturer ID `0x5449`.

`battery.py` samples it on its **own thread** at 2 Hz. The 30 Hz flight loop
never performs an I2C transaction and never blocks on one — it reads whatever
the last completed sample was. A hung bus costs a stale battery number and
nothing else. The reading arrives as `telemetry["battery"]`.

### Current state: `fault: "open_shunt"` — VIN− is not connected

Measured: the shunt amplifier rails at **exactly full scale at every PGA
setting** (40/80/160/320 mV) while the bus-voltage input reads **exactly
0.000 V at both ranges**. That is VIN+ driven with VIN− floating — the load
side of the shunt is open, so the chip never sees the pack.

The module reports this as a named fault with `voltage_v: null`, and **does
not report 0.0 V**. A battery monitor showing a confident zero looks exactly
like a flat pack, which is the one reading you must not fake.

### On the dashboard

The pack voltage is in the flight bar, **directly below SET LEVEL**, in the
same column. It shows volts, and beneath that the cell count, volts per cell
and current.

Colour is by volts **per cell**, not pack volts - 11.1 V is a healthy 3S and a
dangerously flat 4S, so pack voltage alone cannot be coloured meaningfully:

| Per cell | Colour | Meaning |
| --- | --- | --- |
| >= 3.8 V | green | healthy |
| >= 3.5 V | amber | getting low, land soon |
| < 3.5 V | red | land now |

When the reading cannot be trusted it shows **NO READING** and the reason
rather than a number - today that is `VIN- not connected`. It will never show
`0.0 V` for a wiring fault, because that is indistinguishable from a flat
pack at a glance.

Verified on the real dashboard both ways: the fault state as it stands, and
the numeric state with a synthetic 11.10 V 3S reading, which rendered amber at
3.70 V/cell as intended.

### Wiring it — read this before connecting the pack

**The 0.1 Ω shunt cannot carry motor current.** At 3.2 A it already drops
320 mV and dissipates 1 W, which is the full-scale limit; at 10 A it would
dissipate 10 W and destroy itself. Do **not** put it in series with the
ESCs.

For pack voltage — which is what a "volt meter" needs — tie **VIN+ and VIN−
together** to battery positive:

| INA219 pin | Goes to |
| --- | --- |
| `VCC` | Pi **pin 1** (3.3 V) |
| `GND` | Pi **pin 6** (GND) — and this must be common with battery negative |
| `SDA` | Pi **pin 3** |
| `SCL` | Pi **pin 5** |
| `VIN+` | Battery **positive** |
| `VIN−` | Battery **positive** as well — jumpered to VIN+ |

That gives pack voltage with zero risk and no current reading. The
bus-voltage input tops out at **26 V**, so it is fine to 6S.

To measure current as well, VIN− goes to the load's positive instead — but
only for a load under about 3 A. Not the motors.

---

## Power supply — load tested 20 Sep

The upgraded supply **passes**. At a genuine 2.4 GHz on all four cores with
memory and I/O load, for 150 s: `throttled=0x0` throughout, no undervoltage,
no frequency capping, 62.6 °C peak.

Two caveats that matter more than the pass does.

**It does not negotiate USB-PD.** `usbpd_power_data_objects` is all zeros —
there is no PD contract at all. The `max_current = 3000` mA the Pi reports
comes from `usb_max_current_enable=1` in `config.txt`, which tells the
firmware to *allow* 3 A; it is not a measurement and not a negotiation. This
is exactly what a buck converter looks like, and it means the 1.6 A USB
budget is still an assertion rather than something the supply has agreed to.

**The clock is still capped at 1.7 GHz.** `seayou-cpu-cap.service` is enabled
and pins `scaling_max_freq` to 1700000 at boot. It was added 13 Sep for the
*old* supply, from rail measurements: 1.7 GHz held 4.930 V, 1.8 GHz dipped to
4.876 V against a ~4.80 V brownout. The first run of this test therefore
measured nothing useful — 1.7 GHz is 71% of full clock. It was re-run with
the cap lifted, and **the cap was put back afterwards**; removing it
permanently is your call:

```bash
ssh pi@drone.local 'sudo systemctl disable --now seayou-cpu-cap'
```

Before doing that, note what "passed" actually means here. The Pi's own
undervoltage detector only trips near **4.63 V**, whereas the 13 Sep cap was
set on millivolt-level rail readings in the 4.87–4.94 V range — readings
`get_throttled` would never have flagged either. So "no undervoltage flag at
2.4 GHz" is a genuinely weaker statement than the measurement the cap was
based on. **The INA219, once wired, is the instrument that would settle it**
— put it on the 5 V rail and you can compare like for like.

---

## Low-battery failsafe and return to home — 20 Sep

Runs **entirely on the Pi**, inside the same 30 Hz loop as the rest of the
autonomy. It reads the INA219 through telemetry the Pi already has and
commands the Pi's own mission code, so a pack that goes flat with the ground
station gone is still brought down.

### The cells

Molicel **INR-21700-P42A**, from the manufacturer's data sheet
(INR21700P42A-01 Rev 0.2, section 4):

| | |
| --- | --- |
| Nominal voltage | 3.6 V |
| End of charge | 4.20 +/- 0.05 V |
| **End of discharge** | **2.5 V** |
| **Max continuous discharge** | **45 A** |
| Internal resistance | <= 15 mOhm (AC 1 kHz, fresh) |
| Rated capacity | 4.0 Ah minimum |

Note the part number: the **"P42A" is 4.2 Ah of capacity, not 42 A**. The
current rating is 45 A. These are Li-ion, not LiPo - the floor is 2.5 V/cell,
not 3.0 - but 2.5 V is where the cell is empty, not a number to fly to.

### Thresholds

Per cell, measured under flight load, in `battery_guard.py`:

| Level | V/cell | What happens |
| --- | --- | --- |
| `warn` | 3.55 | Dashboard banner. **Nothing automatic.** |
| `return` | 3.40 | Flies home and lands there. No fix -> lands where it is. |
| `land_now` | 3.15 | Comes down immediately, abandoning any return. |
| — | 2.50 | Data-sheet floor. Never reached. |

### Why it is not just a comparison

15 mOhm per cell means the pack sags hard: 20 A per cell is 0.30 V, 45 A is
nearly 0.7 V. A cell resting at 3.7 V reads 3.0 V during a punch-out. So:

- The voltage is **low-pass filtered** (5 s) and a threshold must be held for
  **3 s** before anything happens. A 2 s sag to 3.10 V/cell does nothing.
- Escalation **latches**. Voltage recovers when the throttle comes off, which
  would otherwise cancel a return, let it climb, and trigger it again.
- Cell count is **explicit** (`PACK_CELLS = 3`), not guessed - a 3S at 3.3 V
  and a 4S at 2.5 V are both about 10 V. A reading impossible for that count
  is rejected rather than acted on.

**Measured response**: 11.5 s from a step to 3.30 V/cell; 7.5 s on a realistic
discharge after the pack truly crosses the threshold.

### What it will not do

- **Act on the ground.** `land()` commands throttle, so running it on a drone
  sitting in the grass spins the motors up. Nothing below 1 m.
- **Act with no barometer.** Every descent needs height. Without it, it warns
  loudly and does nothing else.
- **Hold the pilot out.** A stick input aborts a battery return like any other
  mission. The guard is latched, so it comes back 15 s after they go passive -
  authority to the pilot without abandoning the aircraft.
- **Fire on a missing sensor.** An I2C fault is not evidence of a flat pack.

### RTH needs GPS; the fallback does not

`return_home()` flies to the last position the aircraft sat on the ground with
a fix. **With no fix - which is today - it lands where it is instead.** So the
protection against running out mid-flight works right now; only the "fly back
to where you launched" part waits on the GPS.

### Tests

53 checks, no aircraft needed:

```bash
ssh pi@drone.local 'cd /home/pi/dashboard && python3 test_battery_guard.py'
ssh pi@drone.local 'cd /home/pi/dashboard && python3 test_battery_failsafe_integration.py'
```

The first covers the decision (sag rejection, latching, nonsense rejection,
missing sensor). The second covers the action against real `Mission` code:
nothing on the ground, return in the air, `land_now` overriding a return, the
pilot grace period, and no blind descent without a barometer.

---

## Open items

Ranked by what blocks flying.

- [ ] **Multimeter on the GPS module — power, then TX to Pico GP5 (pin 7).**
      The firmware is done; the receiver is sending zero bytes. See *Start here*.
- [ ] **Re-run SET LEVEL on a known-flat surface.** Pitch offset jumped to 8.43°
      but the aircraft still reads +6.89° at rest — something is off. Everything
      else depends on this being right.
- [ ] **Tethered takeoff-and-hold, 10 seconds hands-off.** The measurement that
      settles whether the drift is real, and gives you HOVER LEARNED too.
- [ ] **Connect the INA219's VIN−.** Currently floating, so it reports
      `open_shunt` rather than a voltage. Jumper it to VIN+ for voltage-only.
- [ ] **Compass calibration handshake never confirms.** Fails 5 attempts every
      start, then proceeds anyway. If the Pico's LED is solid rather than
      slow-blinking, it needs a power cycle before flight.
- [ ] **Separate 5 V rail for the LTE dongle** before it goes on the airframe.
- [ ] **`usb-modeswitch` on the Pi** or the dongle stays a CD-ROM.
- [ ] **Push the two Vercel files.**
- [x] ~~Decide on the GPS firmware patch~~ — done 20 Sep, and it was four bugs,
      not two. Flashed. See *GPS: resolved in firmware*.
- [x] ~~Test the existing GPS~~ — the Pico now measures this itself and reports
      zero bytes received at every baud rate.

### Known defects, not fixed

**`MAX_SPEED_MS` is not a limit.** `tilt_per_ms = MAX_TILT_DEG / MAX_SPEED_MS`
is an open-loop guess that 8° equals 3 m/s. Nothing measures actual speed —
`step()` reads position, attitude and baro only — and **there is no speed or
course field anywhere in the Pico packet**. Drag physics puts a small quad
nearer 4–8 m/s at 8°, i.e. faster than the documented cap, which would also
overshoot the 2.5 m arrival radius. Fixable inside `mission.py` by
differentiating GPS position, with no firmware change — but it needs a working
GPS first.

**The NMEA parser is brittle.** Fixed character offsets, and it assumes HDOP is
exactly 4 characters. Even a correct module may parse intermittently.

**The Pi keeps dropping off the hotspot.** The landing takeover now saves the
aircraft, but you lose the link mid-flight. Worth checking signal strength on
the Pi while flying.

### Two lessons worth keeping

**A simulator that does not mirror the aircraft tests nothing.** Three separate
bugs hid behind simulator inaccuracies this week — acceleration used as
velocity, height reported far finer than the hardware can send, and a flight
loop that froze when the socket dropped. All three now match the aircraft.

**"Stop commanding" means throttle 0 means it falls.** Five instances so far.
Any new mission state has this bug by default.
