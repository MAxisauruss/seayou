# What is in this repository

The dashboard is at the root. Everything that makes the aircraft fly, and
everything that serves the dashboard, now lives alongside it.

| Path | What it is |
|---|---|
| `src/`, `public/`, `index.html` | The React dashboard. Built with `npm run build`. |
| `drone/` | What runs on the drone's Raspberry Pi 5: the Pico bridge and web server (`server.py`), the outbound agent (`drone_agent.py`), GPS (`gps.py`), battery monitoring and the low-pack failsafe, autonomous flight (`mission.py`), and the original pilot dashboard in `drone/static/`. |
| `groundstation/` | The laptop-side ground station the drone dials into: `server.py`, the one-click `launcher.py`, `auth.py`, and `sim_drone.py` - a simulator that flies a full waypoint run with no aircraft present. |
| `firmware/` | The Pico flight controller, in C++: `fc_quad_gt_cc/` is the flight code, plus the compass-calibration, GPS-sniffer and motor-test utilities. The `.uf2` files are the flashable builds. |
| `docs/` | The handover document, the level-calibration guide and the V4 wiring diagram. |

## Running it

```bash
npm install && npm run dev          # dashboard, proxied to drone.local
```

```bash
python groundstation/server.py --port 8090 --webapp dist
```

```bash
python groundstation/sim_drone.py --ground-station ws://127.0.0.1:8090/drone --token <printed token>
```

The simulator is the fastest way in: it flies a real waypoint run, reports
GPS, battery and attitude, and needs no hardware.

## Two things worth knowing before you change anything

**There is one control slot.** The Pi keeps a single shared control state
across every connected client, so the dashboard opens exactly one socket
and sending is opt-in (`takeControl()` in `src/services/droneLink.ts`). Two
senders would fight over the throttle thirty times a second.

**Never publish a running control-capable instance.** The source being
public is fine. A tunnel pointed at a ground station with authentication
off is not. See `groundstation/PUBLIC_MIRROR.md`.
