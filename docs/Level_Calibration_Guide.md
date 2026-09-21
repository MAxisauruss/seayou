# Level Zero-Point Calibration

Sets the drone's idea of "level" to the *frame*, not to the sensor board, so a
crookedly-mounted MPU6050 stops making the drone drift.

Press **SET LEVEL** on the dashboard with the drone standing still on a level
surface. The Pico measures its own tilt for 2 seconds, stores the negative of
that as a permanent zero point in flash, and adds it to every attitude reading
from then on. It survives power cycles. You only need to redo it if you move
or remount the sensor board.

---

## Why it has to happen on the Pico

The trim maths already existed in the firmware — `roll_offset_angle` and
`pitch_offset_angle` in `fc_quad_gt_cc.cpp` build a rotation quaternion that
gets applied to the EKF output before the PID ever sees it. Nothing was ever
filling those in with anything but zero.

It could not be done from the Pi side, though. The telemetry packet encodes
each quaternion component as `int(q * 100) + 100` — one byte each. Near level
that works out to about **1.15° per step**, and `int()` truncates toward zero
on top of that. You cannot level anything to better than about a degree from
the far end of that link. On the Pico the same numbers are full `float`s, so
the capture lives there and only the *command* comes from the Pi.

---

## What changed

| File | Change |
|---|---|
| `Raspberry Pi Pico/fc_quad_gt_cc/fc_quad_gt_cc.cpp` | `Level_Capture_Run()`, flash save/load, `'L','C'` and `'L','R'` commands, telemetry bytes 36 and 58–61 |
| `Raspberry Pi Pico/fc_quad_gt_cc/CMakeLists.txt` | links `hardware_flash`, `hardware_sync` |
| `Raspberry Pi 5/dashboard/server.py` | accepts both packet lengths, decodes the level block, `/level_cal` route + WebSocket message |
| `Raspberry Pi 5/dashboard/static/*` | SET LEVEL / ERASE buttons and status readout |

Backups of the originals are alongside each file as
`*.bak-pre-level-capture`.

**The telemetry packet grew from 59 to 63 bytes.** The Pi side accepts *both*
lengths, so it keeps working against a Pico you haven't reflashed yet — that
shows up as the level panel simply not appearing, not as a dead link. The
reverse is not true: new firmware with old `server.py` will desync. Update the
Pi first, then flash the Pico.

---

## Deploying it

**1. Update the Pi (do this first).**

Copy the changed dashboard files to the Pi and restart the service:

```bash
sudo systemctl restart seayou-dashboard
```

Load the dashboard. Everything should look exactly as before — no level panel
yet, because the Pico is still on the old firmware. If telemetry has stopped,
stop here; something else is wrong.

**2. Flash the Pico — no BOOTSEL button needed.**

`picotool` is now installed on the Pi (`sudo apt install picotool`), and it can
put the Pico into BOOTSEL over USB by itself. The whole flash is remote:

```bash
scp "Raspberry Pi Pico/flight_controller_level.uf2" pi@drone.local:/home/pi/
```

Then on the Pi, one step at a time:

```bash
sudo systemctl stop seayou-dashboard
```

```bash
sudo picotool reboot -f -u
```

```bash
sudo picotool load /home/pi/flight_controller_level.uf2 -v -x
```

```bash
sudo systemctl start seayou-dashboard
```

`-v` verifies the write, `-x` reboots into the application afterwards. Stopping
the dashboard first matters — it holds `/dev/pico` open and picotool can't
claim the device otherwise.

`picotool load` only programs the sectors the UF2 actually contains, so **the
stored zero point in the last flash sector survives a reflash.** That is worth
knowing both ways: your calibration is not lost when you update firmware, and
if you ever want a genuinely clean slate you have to press ERASE.

The old working image is on the Pi as `flight_controller_ROLLBACK.uf2` (and
locally as `flight_controller.uf2`) — same three commands to go back.

The manual route still works if picotool ever fails: unplug the Pico, hold
**BOOTSEL**, plug it back in, and it mounts as a drive called `RPI-RP2`
(on the Pi that lands at `/media/pi/RPI-RP2*`). Copy the `.uf2` onto it.

**3. Confirm.** The **LEVEL ZERO-POINT** panel now appears in the dashboard's
telemetry column, showing `0.00° / 0.00°`. That is a Pico that has never been
levelled — which is exactly how it behaved before any of this existed.

---

## Running the calibration

1. Drone on a surface you have actually checked with a spirit level (a phone
   level app is fine). **The surface being level is the whole calibration** —
   the Pico cannot tell the difference between a crooked sensor and a sloped
   table, and it will happily store the slope.
2. **Props off. Disarmed.** The firmware refuses to run if throttle isn't zero.
3. Let it sit for a few seconds so the EKF settles.
4. Press **SET LEVEL**. Do not touch the drone or the bench.
5. Status goes `Settling… → Measuring… → Captured and saved` (~3 s), and
   *Stored offset* changes to the numbers it found.

Anything you bump it with aborts the capture rather than storing a bad value —
you'll get *Aborted – the drone moved*. Just press it again.

Bench alternative, no dashboard needed:

```bash
curl -X POST localhost:8080/level_cal -H 'Content-Type: application/json' -d '{"action":"start"}'
```

```bash
curl localhost:8080/level_cal
```

---

## Bench test (do this before you fly it)

**Test 1 — it actually zeroes.** *(passed on hardware 2026-08-31)*
Note the ROLL and PITCH readouts before pressing SET LEVEL. Press SET LEVEL.
After it saves, both should read `0.0°`, and *Stored offset* should show
roughly the starting numbers with the signs flipped. Measured: started at roll
`9.4°` / pitch `−4.2°`, stored `−10.23° / +4.54°`, ended at `0.0° / 0.0°`.

The stored offset won't exactly match the sign-flipped readout, and that is
expected — the dashboard's numbers are quantized to ~1.15° steps while the
Pico measured at full float precision.

**Test 6 — it holds at any heading.**
Calibrate, then rotate the drone 90° on the spot (flat, don't tilt it) and
check ROLL and PITCH are still near zero. This is what the body-frame fix
buys you; before it, a −35° heading was enough to leave a ~5° residual.

**Test 2 — it survives a power cycle.**
Power the drone down completely, power it back up, reconnect the dashboard.
*Stored offset* must still show the same numbers. If it reads `0.00° / 0.00°`
again, the flash write didn't take.

**Test 3 — it's a real correction, not a frozen number.**
Lift one arm of the drone by a couple of centimetres. The ROLL/PITCH readouts
must move normally and the artificial horizon must tilt. Put it back down and
they return to ~0. If the readouts are stuck, something is wrong — do not fly.

**Test 4 — the abort guard works.**
Press SET LEVEL and immediately nudge the drone. You should get *Aborted – the
drone moved*, and *Stored offset* must be unchanged.

**Test 5 — the safety interlock.**
Arm the drone (props off, throttle at zero). SET LEVEL and ERASE should both
grey out.

---

## Undoing it

**ERASE** puts the stored zero point back to `0.00° / 0.00°` and saves that.
The drone goes back to trusting the sensor board's own idea of level.

To back out completely, flash `flight_controller.uf2` and restore the
`.bak-pre-level-capture` files on the Pi side. The stored zero point stays in
the Pico's last flash sector but nothing reads it.

---

## Limits worth knowing

- **Range is ±15°.** More than that and the capture refuses, because at that
  point it isn't a mounting tolerance — the board is bolted on crooked or the
  drone is on a slope. Fix the physical problem instead.
- **The manual trim bytes still work.** They're added on top of the stored
  zero point, so the dashboard's trim (currently fixed at 25 = 0°) is still
  available as an in-flight fine adjustment.
- **The correction is applied in the body frame** (`q_leveled = q_ekf ⊗
  q_level_rot`, a right-multiply), so it holds at any heading. The original
  firmware left-multiplied, which applies the correction in the *reference*
  frame — that only agrees with a body-frame tilt at zero yaw. Measured on the
  bench at yaw −35°, a −10.23°/+4.54° correction left a 4.4°/5.1° residual
  instead of zero. Swapping the operand order fixed it to 0.0°/0.0°. This also
  changes how the manual trim bytes behave — they are now relative to the
  airframe, which is what trim should mean.
- **The gyro has no bias calibration.** The 2000-sample sweep in `EKF_Init()`
  is commented out, so `gyro_cal[]` is all zeros and `wx/wy/wz` carry the raw
  MPU6050 bias — several °/s even sitting on a bench. The level capture works
  around this by learning the at-rest reading during its settle second and
  testing deviation from that, rather than absolute rate. The underlying gap is
  untouched and pre-dates this work; it feeds the EKF, so re-enabling it changes
  flight behaviour and needs its own testing.
- **Flash writes stall the USB link** for a few tens of milliseconds while
  interrupts are off. Harmless — it only ever happens with the motors off.
