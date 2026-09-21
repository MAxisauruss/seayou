"""
Drowning detection running ON THE PI 5, as a separate process.

This is the onboard half of the rescue behaviour: the drone must be able to
spot someone in the water and interrupt its own flight plan WITHOUT asking
the ground station. If detection lived on the laptop, a dropped link would
mean the aircraft flies past a drowning person - which defeats the point of
putting a Pi 5 on the airframe in the first place.

--------------------------------------------------------------------------
Why a separate process and not a thread
--------------------------------------------------------------------------
The flight loop is asyncio on one thread, exchanging serial packets with
the Pico at 30 Hz. Neural network inference holds the GIL for hundreds of
milliseconds at a time, so running it in a thread inside the agent would
stall that loop - the control link would stutter every time a frame was
processed. A separate process has its own GIL and gets descheduled by the
kernel instead, and it is renice'd so the flight loop always wins a
contested core.

--------------------------------------------------------------------------
Protocol
--------------------------------------------------------------------------
The agent owns the camera (only one process can), so frames come in here
rather than being grabbed independently:

    stdin   4-byte big-endian length, then that many bytes of JPEG
    stdout  one JSON object per line, per frame processed

Backpressure is handled by the agent, which simply does not send a new
frame until this one has been answered. That means a slow model lowers the
detection rate rather than building an ever-growing queue of stale frames -
which for this job is exactly right, since acting on a two-second-old
detection is worse than acting on a fresh one a moment later.
"""
import argparse
import json
import os
import struct
import sys
import time

# Classes the model was trained on. ALERT_CLASS is the one that changes
# how the aircraft flies, so it is named explicitly rather than inferred.
ALERT_CLASS = "Drowning"


def read_frame(stream):
    """Read one length-prefixed JPEG, or None at end of stream."""
    header = stream.read(4)
    if len(header) < 4:
        return None
    (length,) = struct.unpack(">I", header)
    # A wildly wrong length means the stream has desynchronised; better to
    # stop than to try to allocate it.
    if length <= 0 or length > 16 * 1024 * 1024:
        return None
    buf = stream.read(length)
    if len(buf) < length:
        return None
    return buf


def emit(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="On-Pi drowning detection")
    ap.add_argument("--model", required=True, help="path to best.pt (or an exported dir)")
    ap.add_argument("--imgsz", type=int, default=320,
                    help="inference size. 320 is roughly 4x faster than 640 on a "
                         "Pi 5 CPU and is ample for a person-sized object from "
                         "a few metres up")
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--track", dest="track", action="store_true", default=True,
                    help="track detections across frames instead of treating each "
                         "frame independently (default on)")
    ap.add_argument("--no-track", dest="track", action="store_false")
    ap.add_argument("--tracker", default="bytetrack.yaml",
                    choices=["bytetrack.yaml", "botsort.yaml"],
                    help="bytetrack is fast and is the right default on a Pi. "
                         "botsort additionally compensates for CAMERA motion, "
                         "which is the correct answer to a shaky airframe - but "
                         "it costs noticeably more CPU. Watch the reported fps.")
    ap.add_argument("--nice", type=int, default=10,
                    help="process niceness. Positive keeps the flight loop "
                         "ahead of inference on a contested core")
    args = ap.parse_args()

    # Lower our own priority before loading anything heavy.
    try:
        os.nice(args.nice)
    except (AttributeError, OSError):
        pass  # not POSIX, or not permitted - not fatal

    try:
        from ultralytics import YOLO
        import numpy as np
        import cv2
    except ImportError as e:
        emit({"type": "fatal", "error": f"missing dependency: {e}"})
        return 1

    try:
        model = YOLO(args.model)
        names = model.names
    except Exception as e:
        emit({"type": "fatal", "error": f"could not load model: {e}"})
        return 1

    emit({"type": "ready", "classes": names, "imgsz": args.imgsz, "conf": args.conf,
          "alert_class": ALERT_CLASS,
          "tracking": args.tracker if args.track else None})

    stdin = sys.stdin.buffer
    frames = 0
    total_ms = 0.0

    while True:
        jpeg = read_frame(stdin)
        if jpeg is None:
            break
        try:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                emit({"type": "frame", "error": "undecodable jpeg"})
                continue

            t0 = time.monotonic()
            if args.track:
                # persist=True keeps object identities across calls, which is
                # what makes this a tracker rather than a detector run twice.
                #
                # This is deliberately used INSTEAD of stabilising the video.
                # Stabilisation warps pixels to make the picture look steady;
                # a per-frame detector does not care where the horizon is. The
                # problem a shaky airframe actually causes downstream is
                # FLICKER - the same person detected, missed, detected again -
                # and the fix for that is temporal association, not warping.
                # A tracker also gives something a detector cannot: "this
                # person has been in distress for N seconds", which is a far
                # better basis for committing the aircraft than one frame.
                res = model.track(frame, imgsz=args.imgsz, conf=args.conf,
                                  persist=True, tracker=args.tracker,
                                  verbose=False)[0]
            else:
                res = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                                    verbose=False)[0]
            ms = (time.monotonic() - t0) * 1000.0
            frames += 1
            total_ms += ms

            dets = []
            for b in res.boxes:
                xyxyn = b.xyxyn[0]
                dets.append({
                    "label": names[int(b.cls[0])],
                    "conf": round(float(b.conf[0]), 3),
                    # Stable across frames when tracking is on, so the agent
                    # can tell "same person, still in trouble" from "a new
                    # detection". None when tracking is off or the tracker
                    # has not yet committed to an id.
                    "id": int(b.id[0]) if getattr(b, "id", None) is not None else None,
                    # Normalised centre, so it stays meaningful whatever the
                    # camera resolution is set to. Used later to steer the
                    # aircraft towards whoever was spotted.
                    "cx": round(float((xyxyn[0] + xyxyn[2]) / 2), 4),
                    "cy": round(float((xyxyn[1] + xyxyn[3]) / 2), 4),
                    # Fraction of frame area - a crude proximity cue.
                    "area": round(float((xyxyn[2] - xyxyn[0]) * (xyxyn[3] - xyxyn[1])), 4),
                })

            alert = max((d["conf"] for d in dets if d["label"] == ALERT_CLASS), default=0.0)
            emit({
                "type": "frame",
                "t": time.time(),
                "ms": round(ms, 1),
                "avg_ms": round(total_ms / frames, 1),
                "det": dets,
                # Pulled out so the agent does not have to know class names.
                "alert": alert,
            })
        except Exception as e:
            emit({"type": "frame", "error": f"{type(e).__name__}: {e}"})

    emit({"type": "stopped", "frames": frames,
          "avg_ms": round(total_ms / frames, 1) if frames else None})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
