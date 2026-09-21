# Getting the drone off WiFi and onto the internet

Written for the SeaYou drone as it stands: a Raspberry Pi 5 running
`drone_agent.py`, dialling out to a ground station on your PC.

---

## The good news: the hard part is already done

The old setup needed the drone to be **reachable** — you typed
`drone.local:8080` and connected *in* to it. That only works on a network
you control.

The new setup has the drone dial **out**. That difference is what makes
mobile data possible at all, because **mobile networks put you behind
CGNAT** — your SIM gets a private address shared with thousands of other
customers, and nothing on the internet can open a connection to it. Dialling
out sidesteps that completely.

So the drone side is mostly solved. What's left is the other end: **your
ground station now needs to be reachable**, and if your PC is on home WiFi
it has the same problem in reverse.

---

## Step 1 — Pick how the Pi gets online

Three options, cheapest first.

### Option A — Phone hotspot *(free, do this first)*

Nothing to buy. Proves the whole chain works before you spend money.

1. On your phone: turn on **Personal Hotspot / Mobile Hotspot**.
2. On the Pi, add the hotspot as a known network:
   ```bash
   sudo nmcli device wifi connect "YourHotspotName" password "yourpassword"
   ```
3. Check it worked:
   ```bash
   ping -c3 1.1.1.1
   ```

**Limitation:** the phone has to be near the drone and stay on. Fine for
testing, not for range. Also note your Pi's WiFi is now the *uplink*, so
you can't also be on the lab WiFi.

### Option B — USB 4G dongle *(~R400–900, easiest hardware)*

A USB LTE modem (Huawei E3372 and similar) plugs into a Pi 5 USB-A port.
Most present themselves as a USB network adapter and Just Work.

1. Plug it in, then check the Pi sees it:
   ```bash
   ip link
   ```
   Look for a new `eth1`, `usb0`, or `wwan0`.
2. If it appears, you're usually already online. Test with `ping -c3 1.1.1.1`.
3. Buy a **data-only SIM**. Check the APN your network needs — most
   South African networks auto-configure, but if not you set it manually
   (Step 2).

### Option C — 4G HAT *(~R1200+, best for real flying)*

A HAT (SIM7600, Waveshare etc.) sits on the GPIO header — no dangling USB,
better antennas, and it can be powered from the same rail.

⚠️ **Watch the power budget.** Your Pi already browns out on the buck
converter (it's in the project notes, still unresolved). An LTE modem pulls
**2 A peaks** while transmitting. Sort the power supply out *before* adding
this, or you'll get random reboots mid-flight and blame the software.

---

## Step 2 — Set the APN (only if it didn't auto-connect)

```bash
sudo nmcli connection add type gsm ifname "*" con-name lte apn "internet"
sudo nmcli connection up lte
```

Replace `internet` with your network's APN:

| Network | APN |
|---|---|
| Vodacom | `internet` |
| MTN | `internet` |
| Telkom | `internet` |
| Rain | `internet` |

Check which interface is actually carrying traffic:

```bash
ip route get 1.1.1.1
```

---

## Step 3 — Make the ground station reachable

This is the real remaining problem. Your PC on home WiFi has no public
address either.

### Recommended: Tailscale — free, no port forwarding, works behind CGNAT on *both* ends

It puts the Pi and your PC on a private encrypted network as if they were
side by side. This is by far the least painful option and it's what I'd use.

On **both** the Pi and the PC:

1. Make a free account at `tailscale.com`.
2. On the Pi:
   ```bash
   curl -fsSL https://tailscale.com/install.sh | sh
   ```
   ```bash
   sudo tailscale up
   ```
   It prints a link — open it and sign in.
3. Install the Windows client on your PC and sign in with the same account.
4. Find your PC's Tailscale address (looks like `100.x.y.z`).
5. Point the drone at it:
   ```bash
   python3 drone_agent.py --ground-station ws://100.x.y.z:8090/drone --token YOURTOKEN
   ```

Now it works from anywhere with signal, no router configuration, and the
traffic is encrypted end to end.

### Alternative: port forwarding

Forward port 8090 on your home router to your PC. Free, but: you need a
static or dynamic-DNS address, it exposes the ground station to the whole
internet, and **it will not work if your own ISP uses CGNAT**. Only do this
with logins enabled and preferably TLS in front.

---

## Step 4 — Start the agent automatically at boot

You won't have a keyboard on the drone in a field.

```bash
sudo nano /etc/systemd/system/seayou-agent.service
```

```ini
[Unit]
Description=SeaYou drone agent
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/dashboard
ExecStart=/usr/bin/python3 /home/pi/dashboard/drone_agent.py \
  --ground-station ws://100.x.y.z:8090/drone --token YOURTOKEN --video-fps 4
Restart=always
RestartSec=5
StandardOutput=append:/home/pi/dashboard/agent.log
StandardError=append:/home/pi/dashboard/agent.log

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload && sudo systemctl enable --now seayou-agent
```

⚠️ **`seayou-agent` and `seayou-dashboard` both want `/dev/pico`.** Only one
can run at a time. Disable the old one before enabling this:

```bash
sudo systemctl disable --now seayou-dashboard
```

To go back to the original Pi-hosted setup, reverse those two commands.

---

## Step 5 — Turn the video down, seriously

This is where the money goes. The camera streams MJPEG; at 1280×720 and
30 fps that's roughly **5–10 Mbit/s**, which is about **1 GB every 15
minutes**.

`drone_agent.py` already rate-limits it. On mobile data, drop it hard:

```bash
--video-fps 3
```

Rough figures at 720p:

| fps | approx. data per 10 min |
|---|---|
| 30 | ~700 MB |
| 10 | ~230 MB |
| 3 | ~70 MB |
| 0 (telemetry only) | ~2 MB |

**Telemetry alone is tiny** — a few kB/s. If you only need to fly and see
instruments, video off costs almost nothing.

---

## What to expect in the air

- **Latency goes up.** LAN is ~2 ms; mobile data is 40–150 ms and
  occasionally much worse. Position control tolerates that fine. Manual
  stick flying will feel noticeably laggy — this is not a setup for precise
  manual control.
- **The link will drop sometimes.** That is handled: the Pi falls back to
  throttle 0 after 0.5 s, and the Pico's own ~1 s failsafe sits underneath.
  **A dropout means the drone stops, not that it flies away** — but it also
  means it descends, so don't fly high over anything you care about until
  you've seen how often your signal drops.
- **The agent reconnects on its own** with backoff, so a brief outage
  recovers without a restart.

---

## Order I'd do this in

1. **Phone hotspot first.** Free, proves the chain, no hardware risk.
2. **Tailscale next.** Removes the reachability problem permanently.
3. **Fix the Pi's power supply** before any LTE hardware — the brownout
   problem is already open, and a modem's 2 A peaks will make it worse.
4. **Then** buy a dongle or HAT.

Don't buy hardware until steps 1 and 2 work. Almost everything that goes
wrong here is networking, not radios.
