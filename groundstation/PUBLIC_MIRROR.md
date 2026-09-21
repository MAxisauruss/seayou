# Putting live telemetry on the Vercel link

How `https://seayou-indol.vercel.app` shows the real drone, while the
flying still happens on the laptop where you can see the aircraft.

---

## Why the drone cannot just connect to the Vercel link

It was the obvious thing to try, and it cannot work. Two separate walls:

1. **Vercel has no WebSocket server.** Its functions are request/response
   only. There is no `/drone` endpoint for `drone_agent.py` to dial into
   and no `/ws` for the page to listen on, and there is no setting that
   adds one.
2. **A `https:` page cannot reach your LAN.** The browser forbids a page
   served over https from opening `ws://192.168.x.x:8090`. It is called
   mixed content, and it is blocked *silently* — so it looks like the
   drone is offline rather than like a rule you broke.

So the public page talks to a **relay**: one https address that forwards
to the ground station on your laptop.

---

## The shape of it

```
Pico ──USB──▶ Pi 5 (drone_agent.py)
                │
                │  WS out over YOUR LAPTOP HOTSPOT       ~2 ms
                ▼
        Laptop: groundstation/server.py
                │
        ┌───────┴─────────────────┐
        │                         │
   YOU FLY HERE              cloudflared tunnel
   http://localhost:8090          │  https://xxxx.trycloudflare.com
   full control                   ▼
   telemetry + video       seayou-indol.vercel.app
                           WATCHES ONLY
```

The drone never touches the internet. It sits on the hotspot talking to
the laptop over a local socket; the laptop makes the single hop out. **So
the mirror cannot add latency to the flying path** — if the tunnel dies,
or the laptop's internet drops, the aircraft does not notice.

---

## Run it - the one-click way

### 1. Put the Pi on the laptop hotspot

On Windows: **Settings → Network & internet → Mobile hotspot → On**. Note
the network name and password it shows.

Then on the Pi, once:

```bash
sudo nmcli device wifi connect "YourHotspotName" password "yourpassword"
```

⚠ **Check the laptop still has internet after turning the hotspot on.**
If the laptop's only connection is Wi-Fi and it is now *hosting* the
hotspot, it may have no internet left — and then nothing can reach Vercel,
including your own browser. Plug into ethernet, or share from a phone, if
that happens. Local flying is unaffected either way; only the public
mirror needs the laptop online.

### 2. Double-click `SeaYouGroundStation.exe`

That is the whole step. It starts the ground station, opens the dashboard
in your browser, starts the public tunnel if it can, and prints one block
with everything in it:

```
  FLY FROM HERE (this machine, full control):
      http://localhost:8090

  ON THE DRONE (the Pi) - this token no longer changes between
  launches, so set it once and it keeps working:
      python3 drone_agent.py \
          --ground-station ws://192.168.137.1:8090/drone \
          --token <the token this window printed>

  PUBLIC MIRROR (read-only - watches, cannot fly):
      https://seayou-indol.vercel.app/?relay=https://xyz.trycloudflare.com&token=...
```

**The drone token no longer changes between launches.** It is generated
once and kept in `seayou_station.json` next to the .exe. That is the
difference that makes this click-and-go: set the Pi up once and it
connects again tomorrow without being touched.

On the **first run only** it also prints a generated `admin` password, at
the bottom where it is still on screen. Write it down. Lost it? Delete
`users.json` next to the .exe and relaunch.

### 3. Run the printed command on the Pi

Copy it out of the window. To make it permanent, put it in the systemd
unit in `CONNECTING_THE_DRONE_TO_THE_INTERNET.md` — the token is stable
now, so that unit keeps working.

### 4. Fly

The browser it opened is the full dashboard: live telemetry, video, the
gamepad, arming, and **FLY TO COORDINATES**.

To fly to a waypoint, type a **LAT**, a **LON** and a **HEIGHT m** into
that panel and press **GO**. `ABORT` stops it, and so does touching a
stick — manual input always wins, with no button to find first. Limits are
enforced on the aircraft, not in the browser: 300 m range, 1–30 m height,
8° maximum tilt, 3 m/s.

### For the mirror, install cloudflared once

```powershell
winget install --id Cloudflare.cloudflared
```

Or drop `cloudflared.exe` next to `SeaYouGroundStation.exe`. Without it
everything else works and the summary just says the mirror is not running.

### Rebuilding the .exe

```bash
cd seayou-main && npm run build
```

```bash
python groundstation/build_exe.py
```

The dashboard is bundled inside the executable, so rebuild it whenever the
dashboard changes.

### Running from source instead

```bash
python groundstation/launcher.py --webapp seayou-main/dist
```

Or the ground station on its own, with no launcher, no tunnel and a fresh
token each time:

```bash
python groundstation/server.py --port 8090 --webapp seayou-main/dist
```

---

## What the mirror can and cannot do

**Can:** attitude, height, GPS, satellites, barometer, link state, the
video feed, and the full autonomous-flight readout — state, target,
distance, bearing, elapsed, and the onboard detector.

**Cannot:** take control, arm, throttle, trim, level-calibrate, or start
or abort a mission.

That is enforced in **two independent places**, on purpose:

* In the deployed page — `seayou-vercel/src/services/droneTelemetry.ts`
  has **no send path at all**. Not a disabled button or a flag that could
  be flipped: there is no `send` in the module, and no arm, throttle or
  gamepad code anywhere in that tree. Grep it.
* On the server — a request carrying the viewer token is given the
  `viewer` role by `current_user()` in `server.py`, and `browser_ws()`
  drops `control`, `level_cal` and `gp_debug` from any non-pilot while
  `mission_handler()` answers 403.

The server-side half is the one that matters. A check that only exists in
the browser is a check anyone can skip with a websocket client — which is
why the ground station enforces it even under `--no-auth`, where everyone
else is a pilot.

The viewer token also only unlocks four paths — `/ws`, `/status`,
`/stream.mjpg`, `/mission` — and `/mission` only for GET.

---

## Deploying the mirror to Vercel

> **Public source, private control.** The flight dashboard's source is in
> this repository and anyone can read it. That is not the same as exposing
> a control-capable dashboard on the internet: flying still needs the
> ground station on your own machine, the drone token it prints, and the
> aircraft on the same network. What must never be published is a running
> instance - a tunnel to a control-capable ground station with auth off, or
> a deployed build wired to one. The mirror avoids exactly that by shipping
> the read-only relay client and a viewer token that cannot arm or fly.


**Deploy `seayou-vercel/`, not `seayou-main/`.** There are two trees and
they are not the same project:

| Folder | What it is |
|---|---|
| `seayou-main/` | The flight dashboard you fly with, served by the ground station. Full control, gamepad, arming. Its SOURCE now lives in this repository (the app at the root), which is public - but do not DEPLOY a control-capable build to a public URL. |
| `seayou-vercel/` | The deployed site. Extracted from `seayou-main (2).zip` (the version behind `seayou-indol.vercel.app`) plus the read-only telemetry. |

`seayou-vercel/` is the zip's own source with three changes:

```
src/services/droneTelemetry.ts   NEW  - read-only relay client
src/pages/LiveFeeds.tsx          live telemetry, gated on ?relay=
vite.config.ts                   removed base: '/my-website/'
```

Everything else is byte-identical to the zip, so it drops into your repo
cleanly.

### The base fix matters

The zip shipped `base: '/my-website/'` in `vite.config.ts`, left over from
a GitHub Pages template. Vercel serves the build at the domain root, so
that prefix makes `index.html` ask for `/my-website/assets/index-….js` and
every asset 404s — **a blank page with nothing on screen to explain it**.
Verified by building it both ways. The site currently deployed does *not*
have the prefix, so whatever is wired to Vercel already differs from this
zip; leave the prefix out.

### Pick your route

**If a GitHub repo auto-deploys it** — copy those three files into the
repo and push.

**If you use the CLI:**

```powershell
npm i -g vercel
```

```bash
cd seayou-vercel && vercel --prod
```

**If you upload through the dashboard** — `npm run build` in
`seayou-vercel` and upload `dist/`.

### Nothing else to configure

The mirror needs no environment variables. With no `?relay=` in the URL
the telemetry module opens no socket at all and every panel keeps its
original demo content — so **deploying this cannot change what a visitor
to the plain link sees**. It only comes alive when opened with a relay.

## When it does not work

| What you see | What it is |
|---|---|
| Mirror says `disconnected`, ground station fine | Tunnel not running, or the URL changed. Re-open with the new `?relay=`. |
| Mirror connects but no telemetry | Drone agent is not connected to the ground station. Check `/status`. |
| `NO CAMERA // TELEMETRY LINK OK` | Normal with no camera, and normal with the simulator. Video is a separate stream. |
| Video stutters on the mirror | Expected — it goes over the internet. Drop it: `--video-fps 4` on the agent. Telemetry is unaffected. |
| Mirror loads but nothing anywhere | Laptop has no internet (see the hotspot warning in step 1). |

---

## Testing it with no aircraft

Everything above works against the simulator:

```bash
python groundstation/server.py --port 8090 --webapp seayou-main/dist
```

```bash
python groundstation/sim_drone.py --ground-station ws://127.0.0.1:8090/drone
```

Then start a mission from the local dashboard and watch it appear on the
mirror. The simulator flies a real waypoint run — it is how the mirror was
verified.
