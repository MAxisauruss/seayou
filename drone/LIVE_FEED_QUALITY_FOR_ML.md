# Giving the detector a decent picture to work with

Research notes, 2026-09-12. The question was "what can we use to stabilise
the live feed so the machine learning has something proper to work with".

**The short answer: stabilisation is the wrong lever.** It fixes the thing
that bothers human eyes and barely touches the thing that hurts a neural
network. The three changes that matter are a shutter setting, a camera
choice, and a tracker — none of which is stabilisation.

---

## Why stabilisation does not help the detector much

A detector looks at **one frame at a time**. It has no idea the horizon
moved since the last frame, and it does not care.

Stabilisation works by warping each frame so the *sequence* looks steady.
That is a post-hoc pixel transform. It does not add information, and to a
per-frame detector it changes almost nothing — it can even *remove*
information by cropping the borders it needs to warp into.

What genuinely degrades detection, in order:

| Problem | What it does to the model | Fixed by stabilisation? |
|---|---|---|
| **Motion blur** | The person becomes a smear. Worst offender. | ❌ Baked in during exposure — nothing downstream can undo it |
| **Rolling-shutter "jello"** | Object shapes are *bent* within the frame | ❌ Frame-level warping moves the bent frame around |
| **Detection flicker** | Same person found, missed, found again | ❌ Not a pixel problem at all |
| Frame-to-frame shake | Almost nothing | ✅ …but this is the least important row |

The literature agrees that blur is the real enemy: *"motion blur caused by
rapid camera movement severely degrades detection accuracy, often resulting
in fragmented tracking trajectories."* Note that most detection work does
not address it at all — it is a known gap, not a solved problem.

---

## Options considered

### 1. Gyroflow — ❌ rejected

Post-processing only. Their own docs have a *Live Feed Stabilization* page
whose content is an explanation of why it does not exist: no standard for
transmitting live IMU data, and the architecture was built for recorded
files. It cannot sit in a live path.

It would also need raw gyro at 200 Hz+; our telemetry carries only the
quaternion at 30 Hz quantised to **~1.15° per step**. Far too coarse.

*Still useful in post* for report footage — which is why video recording was
added (`--record-video`). It would need a firmware change to log real gyro.

### 2. Real-time EIS on the Pi (OpenCV / vidstab) — ❌ not worth it

Technically feasible: ~6–9 ms/frame for 720p 2D stabilisation on a Pi 4B,
so faster on a Pi 5. Roughly 25 fps at 360p has been measured.

But it would **compete with YOLO for the same cores**, and it fixes the
least important row in the table above. Spending 30–40% of the CPU budget
to make the video look nicer to a human, while slowing the detector that
actually needs those cycles, is the wrong trade on this airframe.

### 3. Mechanical gimbal — ⚠️ effective but heavy

A 2-axis brushless gimbal genuinely fixes shake *and* reduces blur (the
sensor is physically still during exposure). But it adds weight, cost, a
power draw, and another thing to fail — on a drone that already browns out
on its own power supply and has an unresolved attitude-stability history.

Not now. Revisit if the airframe becomes stable and there is payload margin.

### 4. Soft-mounting the camera — ✅ cheap, do it

Vibration is the *cause* of both blur and jello. Damping it at the source
helps everything downstream at no CPU cost. Vibration-damping foam or
silicone standoffs between the camera and the frame. Balance the props too
— this is already a known issue on this drone.

---

## What to actually do, cheapest first

### ✅ 1. Cap the shutter speed — implemented

**The single biggest win, and it is free.** Applied in `CameraStream` in
`server.py`:

```python
AE_MAX_SHUTTER_US = 2500     # 1/400 s
AeExposureMode = 1           # "short" - the sport/motion mode
```

Capping exposure at 1/400 s is the standard guidance for freezing a moving
subject. Auto-exposure still runs, it just may not choose a long shutter.

Cost: a darker image, compensated with gain (noise). **In daylight over
water — this drone's entire use case — there is light to spare.**

### ✅ 2. Track instead of stabilise — implemented

`pi_detector.py` now calls `model.track(..., persist=True)` rather than
`predict()`. Ultralytics has ByteTrack and BoT-SORT built in.

This is the *correct* answer to a shaky airframe, because the problem a
shaky feed actually causes downstream is **flicker**, and the fix for
flicker is temporal association, not warping pixels.

It also gives something a detector cannot: **"this person has been in
distress for N seconds"** — a far better basis for committing the aircraft
than a single frame.

- `bytetrack.yaml` *(default)* — fast, good default performance
- `botsort.yaml` — **additionally compensates for camera motion**, which is
  precisely the shaky-airframe case, but costs noticeably more CPU

Start with bytetrack, watch the reported fps, and try botsort if flicker is
still a problem and you have headroom.

### ✅ 3. Buy a global shutter camera — ~$50, do this

You are replacing the lost camera anyway. **Raspberry Pi Global Shutter
Camera** (Sony IMX296, 1.6 MP, C/CS mount).

Global shutter exposes the whole sensor at once, so there is no line-by-line
readout and **jello cannot physically occur**. Raspberry Pi market it for
machine vision specifically because *"even small amounts of distortion can
seriously degrade inference performance"* — which is exactly our problem.

It also does short exposures well (down to 30 µs), which serves change #1.

Trade-off: 1.6 MP, not 12 MP. Irrelevant here — we infer at 320 px.

### 4. Soft-mount it, and balance the props

No code. Biggest return per rand after the camera.

---

## What NOT to do

- **Do not add real-time stabilisation.** It costs the CPU the detector
  needs and fixes the least important problem.
- **Do not raise the inference resolution to compensate** for a blurry
  image. A sharp 320 px frame beats a blurred 640 px one, and costs 4× less.
- **Do not put Gyroflow in the live path.** It cannot go there.

---

## Sources

- [Gyroflow — Live Feed Stabilization](https://docs.gyroflow.xyz/app/advanced-usage/live-feed-stabilization)
- [Gyroflow — GCSV format](https://docs.gyroflow.xyz/app/technical-details/gcsv-format)
- [Raspberry Pi Global Shutter Camera](https://www.raspberrypi.com/news/new-raspberry-pi-global-shutter-camera/)
- [Ultralytics — Multi-Object Tracking](https://docs.ultralytics.com/modes/track)
- [Picamera2 — camera controls](https://github.com/raspberrypi/picamera2/discussions/760)
- [UAV video deblurring and detection](https://arxiv.org/html/2608.15259)
- [Real-time stabilisation on Raspberry Pi](https://github.com/mrnurkhat/RealTimeVideoStabilization)
