# SeaYou ground station

Runs on the PC. Hosts the dashboard; the drone dials **out** to it.

```
Browser ──HTTP/WS──▶ ground station ◀──WS (outbound)── Pi agent ──USB──▶ Pico
                     (this folder)        drone_agent.py
```

## Why this way round

The drone never needs an inbound address. On a LAN that is convenient; on
mobile data it is essential, because carriers put SIMs behind CGNAT where
nothing outside can reach them. A drone that dials out works from any
connection it can get.

## Running it

**The easy way: double-click `SeaYouGroundStation.exe`.** It starts the
ground station, opens the dashboard, starts the public tunnel if
cloudflared is available, and prints the exact command to run on the Pi.
The drone token is kept in `seayou_station.json` next to the .exe and
**does not change between launches**, so the Pi is configured once. See
`PUBLIC_MIRROR.md`. Rebuild it with `python build_exe.py`.

From source, the same thing:

```bash
python groundstation/launcher.py --webapp seayou-main/dist
```

Or the ground station on its own:

```bash
python groundstation/server.py --port 8090 --webapp seayou-main/dist
```

It prints a generated token. Then on the Pi:

```bash
python3 drone_agent.py --ground-station ws://<pc-ip>:8090/drone --token <token>
```

No drone? Use the simulator — same protocol, same telemetry shape:

```bash
python groundstation/sim_drone.py --ground-station ws://127.0.0.1:8090/drone
```

Add `--no-gps` to reproduce the real aircraft's dead GPS.

## What did NOT move

The Pico serial link, the 30 Hz control loop, the failsafes and the flight
logging all stay on the Pi. They need hard timing and must survive the
network dying.

**If the ground station disappears mid-flight**, the Pi does not need it.
Guidance runs onboard in `drone_agent.py`, inside the same 30 Hz loop that
talks to the Pico, and `server.py` applies the staleness failsafe BEFORE
calling the autonomy hook and sends whatever the hook returns — so the
aircraft can fly itself with the network completely gone.

What it does with no link: **an automatic manoeuvre LANDS.** It used to
`abort()`, which means throttle 0, so a hover at 2 m became a 2 m fall the
moment the WiFi hiccupped. Landing keeps the original intent — stop flying
a mission nobody can see or stop — without dropping the aircraft. Pass
`--autonomous-on-link-loss` to keep flying the mission instead.

If there is no barometer it still aborts to throttle 0, because without a
height reference no controlled descent is possible. And underneath all of
it, with nothing commanding at all, the Pico's own ~1 s no-data failsafe
remains. Flight logging continues regardless.

Verified by killing the ground station process mid-hover: the drone came
down 2.96 → 2.63 → 2.14 → 1.31 → 0.00 m on its own.

`server.py` on the Pi is untouched and still works standalone, so the
original Pi-hosted setup at `drone.local:8080` remains as a fallback — it is
the flight-tested path.

## Public mirror

`https://seayou-indol.vercel.app` can show live telemetry **read-only**,
through an https relay in front of this server. The drone is not involved
and stays on the local network, so the mirror cannot add latency to the
flying path. See `PUBLIC_MIRROR.md`.

The mirror can watch everything and command nothing: the viewer token
gives the `viewer` role, and `browser_ws()` drops control from non-pilots
while `mission_handler()` answers 403.

## Takeoff, land, and what the barometer is actually worth

`TAKE OFF` and `LAND` are in the AUTONOMOUS panel, behind the gear icon
for the height. **They need the barometer and nothing else — no GPS fix**,
which is why they are the autonomous features this aircraft can use today
and the waypoint is not.

### The barometer works. It is just coarse.

It is a real BMP388: chip ID checked at init, full Bosch compensation,
pressure oversampling x8. `isBaroPresent` is false only if that chip-ID
read fails, and then `baro` is `null` and the dashboard shows no height.

What limits it is the packet, not the sensor. The firmware sends

```c
uint16_t press_scaled = (uint16_t)(baro_pressure_pa / 10.0f);
```

so the smallest change that can be transmitted is 10 Pa. What that is
worth in metres depends on where you are: **843.7 / pressure_hPa**, which
is 0.83 m at sea level and **0.97 m at the ~1280 m this aircraft actually
lives at** (it reads 868.8 hPa on the bench). The dashboard computes it
from live pressure rather than assuming sea level. A 3 m flight therefore
has about three distinct height values in it. The BMP388 itself resolves
far better; the resolution is thrown away on the wire.

**Confirmed working on the real aircraft 2026-09-14:** 868.8 hPa, 25 °C,
height 0.00 m and steady. Sitting completely still it occasionally
flickers one step — which reads as a 0.97 m jump in height. Expect the
height hold to hunt by about that much; it is measurement noise, not the
controller misbehaving.

Everything downstream is sized for that: `ALT_ARRIVE_M` is 1.0 m,
`LAND_CUTOFF_M` is 0.9 m, and a height hold hunts by about one step. That
is not sloppy tuning — it is the measurement limit.

### Check yours in thirty seconds

1. Power the drone up and open the dashboard.
2. Look at **HEIGHT** in the AUTONOMOUS panel.
   * A number that changes → the barometer is working.
   * `— no barometer` → the BMP388 was not detected. Check the I2C wiring;
     the bus on this aircraft is already noted as marginal.
3. Lift the drone about a metre by hand. The reading should move by about
   one step (0.83 m), not smoothly.

If it never moves off `0.00` while you lift it, the sensor is being read
but is not responding — which is different from not being present, and
worth knowing before trusting a takeoff to it.

### What takeoff and land do

**TAKE OFF** climbs at 0.5 m/s to the height in the settings drawer
(1–10 m, default 2 m), then holds. It refuses if it is already above 1 m,
if the height is outside that band, or if there is no barometer.

**LAND** descends at 0.4 m/s, and below 0.9 m stops trusting the
barometer and eases the throttle to zero over 3 s. **A ramp, not a cut:**
from one barometer step up, cutting the motors is indistinguishable from
dropping it.

**Neither holds position.** Roll and pitch are held level, so wind walks
the aircraft sideways the whole time. Keep a hand on the sticks — moving
one aborts instantly and hands control straight back.

⚠ **ABORT is not LAND.** Abort idles the motors immediately, which from
2 m is a 2 m drop. It is the right behaviour when something is wrong and
you want the aircraft handed back, but `LAND` is the button for coming
down on purpose.

Verified against the simulator with GPS switched off, which is the real
aircraft's current state: climbs, holds, lands, and idles; a stick input
takes a climb back instantly. **Never flown on the real aircraft. Tether
it.**

## Where waypoints actually stand

The chain is complete and works end to end against `sim_drone.py`: a
waypoint runs, arrives, and holds. The NMEA-to-decimal conversion that was
missing is now in the Pi's `server.py` (`_nmea_to_decimal`), so the real
telemetry path carries `gps.lat` / `gps.lon` in decimal degrees.

**What is still blocking it on the real aircraft is the GPS module itself:
it reports 0 satellites and has never produced a fix.** `Mission.start()`
refuses without one, by design — flying to a coordinate you cannot
measure your distance from is worse than not flying at all. Reproduce
exactly what the aircraft does today with:

```bash
python groundstation/sim_drone.py --ground-station ws://127.0.0.1:8090/drone --no-gps
```

So: fix the GPS, and waypoint flight should work. Nothing in software is
waiting on anything else.
