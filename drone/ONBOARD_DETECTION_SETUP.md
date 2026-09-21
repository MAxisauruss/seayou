# Running drowning detection on the Pi 5

The model runs **on the aircraft**, not on the laptop. That is the whole
reason for carrying a Pi 5: the drone has to be able to notice someone in
the water and stop its own flight plan **without asking the ground
station**, because a dropped link must never mean it flies past them.

---

## What happens when it sees someone

1. `pi_detector.py` (separate process) reports a `Drowning` detection.
2. `drone_agent.py` waits for **3 consecutive frames** before believing it —
   one false positive must not stop a mission.
3. The mission switches from `running` to `holding` and **retargets to the
   drone's current position**.
4. The aircraft **hovers over the spot**. It does *not* land, and it does
   *not* cut throttle.
5. The dashboard shows `⚠ DROWNING` and the reason on the mission panel.

It takes **10 consecutive clear frames** to drop the alert — higher than the
3 needed to raise it, on purpose. Someone partly submerged is easy to miss
for a frame or two, and resuming a flight plan while they are still in the
water would be the worst possible failure.

---

## Install on the Pi

```bash
ssh pi@drone.local
```

```bash
sudo apt update && sudo apt install -y python3-opencv libopenblas-dev
```

```bash
pip install --break-system-packages ultralytics
```

⚠️ This pulls PyTorch for ARM — **roughly 1–2 GB and 10+ minutes**. Make sure
the SD card has room (`df -h`).

Copy the trained weights across:

```bash
scp "seayou-main/backend/runs/detect/train/weights/best.pt" pi@drone.local:/home/pi/best.pt
```

Also copy `mission.py` next to the agent, or waypoint guidance stays off:

```bash
scp groundstation/mission.py "Raspberry Pi 5/dashboard/pi_detector.py" "Raspberry Pi 5/dashboard/drone_agent.py" "Raspberry Pi 5/dashboard/server.py" pi@drone.local:/home/pi/dashboard/
```

---

## Run it

```bash
python3 drone_agent.py --ground-station ws://<pc-ip>:8090/drone --token XXXX \
  --detector-model /home/pi/best.pt
```

Leave `--detector-model` off and detection is simply disabled; everything
else works exactly as before.

---

## Speed — what to actually expect

**I have not benchmarked this on your Pi** (it was powered down when this
was written). These are typical Pi 5 figures for YOLOv8n, and the agent
prints the real number to the dashboard as `fps on the Pi` once it runs —
trust that over this table.

| Setup | Rough fps |
|---|---|
| PyTorch, `--detector-imgsz 640` | 1–2 |
| PyTorch, `--detector-imgsz 320` *(default)* | 4–6 |
| **NCNN export, 320** | **8–12** |
| Hailo-8L AI HAT | 30+ |

**320 px is the default and is the right call here.** A person from a few
metres up is a large object in frame; 640 buys accuracy you do not need and
costs about 4× the time.

**Is 4–6 fps enough?** For this job, yes. A drowning person is not moving
fast, and the 3-frame confirmation means a decision takes under a second.

### Making it faster with NCNN

```bash
yolo export model=/home/pi/best.pt format=ncnn imgsz=320
```

Then point the agent at the exported directory instead:

```bash
--detector-model /home/pi/best_ncnn_model
```

Typically about **2× faster** than PyTorch on ARM, same weights.

### If you want it genuinely fast

The **Raspberry Pi AI HAT+ (Hailo-8L)** is the proper answer — 30+ fps and it
offloads the CPU entirely, leaving the flight loop alone. It needs the model
re-compiled into Hailo's format. Worth it if detection becomes central;
overkill if 5 fps does the job.

⚠️ Same power warning as the LTE modem: the Pi already browns out on the
buck converter. Do not add an AI HAT before that is fixed.

---

## Why it will not disturb the flight loop

Two deliberate choices:

**Separate process, not a thread.** Inference holds Python's GIL for
hundreds of milliseconds. In a thread it would stall the 30 Hz serial loop
and make the control link stutter every frame. A separate process has its
own GIL and gets descheduled by the kernel instead.

**Renice'd to +10.** On a contested core the flight loop always wins. The
Pi 5 has 4 cores, so in practice they rarely contend at all.

**Frames are dropped, never queued.** The agent only hands over a new frame
once the last one has been answered. A slow model lowers the detection rate
rather than building a backlog of stale frames — acting on a two-second-old
detection is worse than acting on a fresh one a moment later.

---

## Checking it works

`fps on the Pi` and a frame count appear on the mission panel. On the Pi:

```bash
tail -f /home/pi/dashboard/agent.log
```

Look for `Detector ready: classes={0: 'Drowning', 1: 'Person out of water', 2: 'Swimming'}`.

Watch the CPU while it runs — the flight loop must stay responsive:

```bash
top -d1
```

If `Packet age` on the dashboard starts climbing above ~0.1 s while
detection is running, the model is starving the control loop. Lower
`--detector-imgsz` to 256, or raise `--nice`.

---

## Not yet done

- **Never run on the real aircraft.** Tested only against `sim_drone.py`
  with a faked detection. The detector process itself has never had a real
  camera frame through it on the Pi.
- **The camera is currently not detected** (`No cameras available!`).
  Reseat the ribbon and reboot — the Pi only probes the camera port at boot.
- **Detection does not yet steer toward the person.** It stops and holds
  where the drone already is. `pi_detector.py` reports the detection's
  position in frame (`cx`, `cy`) specifically so that centring on them can
  be added next, but that is a new control loop and needs its own testing.
