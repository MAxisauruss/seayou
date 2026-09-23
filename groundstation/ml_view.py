"""
SeaYou ML view - the drone's camera, with the drowning-detection model's
boxes drawn on it.

    groundstation/.venv-ml/Scripts/python.exe groundstation/ml_view.py

Then open http://localhost:8095 (or the ML VIEW tile on Live Feeds).

WHY THIS IS ITS OWN PROCESS
The model needs PyTorch, Ultralytics and OpenCV - over a gigabyte - and the
ground station deliberately has none of them: it ships as a 15 MB .exe that
has to start on any laptop. So the model runs here, in its own environment
(groundstation/.venv-ml), and reads the video the same way a browser does.
Nothing about the ground station changes, and if this process is not
running the dashboard just shows that the ML view is offline.

WHERE THE VIDEO COMES FROM
By default the ground station's own /stream.mjpg - the drone's camera -
using the READ-ONLY viewer token from seayou_station.json. That token can
watch but cannot arm or fly, so this process could not command the aircraft
even by mistake. --source also takes a webcam index, a video file (looped),
a folder of images, or any RTSP/HTTP stream, so it can be demonstrated with
no drone at all.

WHAT IT DRAWS
The team's trained model (seayou-main/backend/runs/detect/train/weights/
best.pt, YOLOv8n): classes Drowning, Person out of water, Swimming. On its
own validation set it scored precision 0.87, recall 0.80, mAP50 0.87 - good,
not infallible, which is why every box shows its confidence. A Drowning
detection at or above the alert threshold (0.90, the same as the team's
DroneCamera.py) turns the header red.
"""
import argparse
import glob
import json
import logging
import os
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

log = logging.getLogger("ml_view")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

HERE = os.path.dirname(os.path.abspath(__file__))

#: Same threshold the team's DroneCamera.py / BeachCamera.py alert on.
DEFAULT_ALERT = 0.90

#: Boxes below this are not drawn. Lower than the alert threshold on
#: purpose: a demo should show what the model is considering, not only
#: what it is certain of - the confidence on each box says which is which.
DEFAULT_CONF = 0.35

ALERT_CLASS = "Drowning"

#: Box colours, BGR. Chosen to mean something - red is the one that matters
#: - and matched by the dashboard's detection chips (MlView.tsx).
CLASS_BGR = {
    "Drowning": (38, 38, 220),               # red    #dc2626
    "Person out of water": (11, 158, 245),   # amber  #f59e0b
    "Swimming": (233, 165, 14),              # blue   #0ea5e9
}


def find_model():
    """best.pt in either layout: the working tree or the repository."""
    for rel in ("../seayou-main/backend/runs/detect/train/weights/best.pt",
                "../backend/runs/detect/train/weights/best.pt"):
        p = os.path.normpath(os.path.join(HERE, rel))
        if os.path.isfile(p):
            return p
    return None


def viewer_token():
    """The ground station's read-only token - watches, cannot fly."""
    for rel in ("dist/seayou_station.json", "seayou_station.json"):
        p = os.path.join(HERE, rel)
        if os.path.isfile(p):
            try:
                with open(p, encoding="utf-8") as f:
                    return json.load(f).get("viewer_token")
            except (OSError, ValueError):
                pass
    return None


# ---------------------------------------------------------------------------
# Frame sources. Each keeps ONLY the latest frame: a detector that queues
# frames falls further behind every second and ends up showing the past.
# ---------------------------------------------------------------------------

class FrameSource(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.frame = None           # latest BGR frame
        self.seq = 0
        self.status = "starting"

    def publish(self, frame):
        with self.lock:
            self.frame = frame
            self.seq += 1
        self.status = "ok"

    def latest(self):
        with self.lock:
            return self.frame, self.seq


class MjpegSource(FrameSource):
    """An HTTP multipart/x-mixed-replace stream - the ground station's feed.

    Each part is read by its own Content-Length header, which the ground
    station always sends, and decoded by content rather than by assuming
    JPEG. The first version split on JPEG start/end markers instead, and
    showed NOTHING when fed the dashboard's scene images - which turned out
    to be PNGs named .jpg. A part with no Content-Length falls back to
    scanning for the JPEG markers.
    """

    def __init__(self, url, label):
        super().__init__()
        self.url = url
        self.label = label

    def _decode(self, data):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.publish(img)
        else:
            self.status = "receiving data, but it is not an image"

    def run(self):
        while True:
            try:
                with urllib.request.urlopen(self.url, timeout=10) as r:
                    self.status = "connected, waiting for frames"
                    while True:
                        line = r.readline()
                        if not line:
                            raise ConnectionError("stream ended")
                        if not line.startswith(b"--"):
                            continue                      # between parts
                        length = None
                        while True:                       # this part's headers
                            h = r.readline()
                            if not h:
                                raise ConnectionError("stream ended")
                            if h in (b"\r\n", b"\n"):
                                break
                            name, _, value = h.decode("latin-1").partition(":")
                            if name.strip().lower() == "content-length":
                                length = int(value.strip())
                        if length is not None:
                            self._decode(r.read(length))
                            continue
                        # No length: read until the JPEG end marker.
                        buf = b""
                        while b"\xff\xd9" not in buf and len(buf) < 8_000_000:
                            chunk = r.read(4096)
                            if not chunk:
                                raise ConnectionError("stream ended")
                            buf += chunk
                        start = buf.find(b"\xff\xd8")
                        end = buf.find(b"\xff\xd9")
                        if start != -1 and end > start:
                            self._decode(buf[start:end + 2])
            except urllib.error.HTTPError as e:
                # 503 is the ground station saying the drone sends no video.
                body = e.read().decode("utf-8", "replace").strip()[:80]
                self.status = "no video: %s" % (body or "HTTP %d" % e.code)
            except Exception as e:  # noqa: BLE001 - keep retrying whatever it was
                self.status = "cannot reach %s (%s)" % (self.label, e.__class__.__name__)
            time.sleep(2.0)


class CaptureSource(FrameSource):
    """A webcam index, a video file (looped), or an RTSP URL."""

    def __init__(self, src, loop):
        super().__init__()
        self.src = src
        self.loop = loop

    def run(self):
        while True:
            cap = cv2.VideoCapture(self.src)
            if not cap.isOpened():
                self.status = "cannot open %s" % self.src
                time.sleep(2.0)
                continue
            fps = cap.get(cv2.CAP_PROP_FPS) or 0
            delay = 1.0 / fps if self.loop and 1 < fps < 120 else 0.0
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                self.publish(frame)
                if delay:
                    time.sleep(delay)   # play a file at its own speed
            cap.release()
            if not self.loop:
                self.status = "source ended - retrying"
            time.sleep(0.2 if self.loop else 2.0)


class FolderSource(FrameSource):
    """Cycle through a folder of still images - a demo with no camera."""

    def __init__(self, folder, every_s=3.0):
        super().__init__()
        self.paths = sorted(p for p in glob.glob(os.path.join(folder, "*"))
                            if p.lower().endswith((".jpg", ".jpeg", ".png")))
        self.every_s = every_s

    def run(self):
        if not self.paths:
            self.status = "no images in folder"
            return
        while True:
            for p in self.paths:
                img = cv2.imread(p)
                if img is not None:
                    self.publish(img)
                time.sleep(self.every_s)


# ---------------------------------------------------------------------------
# The detector
# ---------------------------------------------------------------------------

class Detector(threading.Thread):
    def __init__(self, source, model_path, conf, alert, width):
        super().__init__(daemon=True)
        from ultralytics import YOLO
        self.source = source
        self.model = YOLO(model_path)
        self.model_name = os.path.basename(model_path)
        self.classes = [self.model.names[i] for i in sorted(self.model.names)]
        self.conf = conf
        self.alert = alert
        self.width = width
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.jpeg = None
        self.jpeg_seq = 0
        self.state = {"detections": [], "fps": 0.0, "infer_ms": None,
                      "frames": 0, "alert": None, "frame_size": None}

    def run(self):
        last_seq = -1
        times = []
        while True:
            frame, seq = self.source.latest()
            if frame is None or seq == last_seq:
                if frame is None:
                    self._publish(self._waiting_card(), [], None, None, None)
                time.sleep(0.05 if frame is not None else 1.0)
                continue
            last_seq = seq

            h, w = frame.shape[:2]
            if self.width and w > self.width:
                frame = cv2.resize(frame, (self.width, int(h * self.width / w)))
                h, w = frame.shape[:2]

            t0 = time.perf_counter()
            res = self.model.predict(frame, conf=self.conf, verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000.0

            dets = []
            alert = None
            for b in res.boxes:
                label = res.names[int(b.cls[0])]
                c = float(b.conf[0])
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
                dets.append({"label": label, "conf": round(c, 3),
                             "box": [round(x1), round(y1), round(x2), round(y2)]})
                if label == ALERT_CLASS and c >= self.alert and (
                        alert is None or c > alert["conf"]):
                    alert = {"label": label, "conf": round(c, 3)}

            annotated = frame.copy()
            for d in dets:
                self._frame_object(annotated, d)
            now = time.time()
            times = [t for t in times if now - t < 3.0] + [now]
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
            self._header(annotated, dets, fps, infer_ms, alert)
            self._publish(annotated, dets, fps, infer_ms, alert, size=(w, h))

    def _frame_object(self, img, d):
        """One box, in a colour that MEANS something, labelled with the class
        and its confidence.

        Drawn here rather than with Ultralytics' results.plot(), whose
        palette is assigned by class index - it painted Person out of water
        pink and Swimming orange, so the colour said nothing, and the
        dashboard's detection chips could not match it.
        """
        x1, y1, x2, y2 = d["box"]
        colour = CLASS_BGR.get(d["label"], (230, 230, 230))
        thick = 3 if d["label"] == ALERT_CLASS else 2
        cv2.rectangle(img, (x1, y1), (x2, y2), colour, thick)
        text = "%s %d%%" % (d["label"], round(d["conf"] * 100))
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = y1 - 6 if y1 - th - 10 > 30 else y1 + th + 8   # keep clear of the header
        cv2.rectangle(img, (x1, ty - th - 4), (x1 + tw + 8, ty + 4), colour, -1)
        ink = (255, 255, 255) if d["label"] == ALERT_CLASS else (15, 15, 15)
        cv2.putText(img, text, (x1 + 4, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ink, 1, cv2.LINE_AA)

    def _header(self, img, dets, fps, infer_ms, alert):
        w = img.shape[1]
        bar = (0, 0, 200) if alert else (20, 14, 5)
        cv2.rectangle(img, (0, 0), (w, 28), bar, -1)
        left = ("DROWNING DETECTED %.0f%%" % (alert["conf"] * 100)) if alert else "SEAYOU ML VIEW"
        cv2.putText(img, left, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        right = "%d object%s  %.1f fps  %.0f ms" % (
            len(dets), "" if len(dets) == 1 else "s", fps, infer_ms)
        (tw, _), _ = cv2.getTextSize(right, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.putText(img, right, (w - tw - 8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 220, 240), 1, cv2.LINE_AA)

    def _waiting_card(self):
        """What to show with no video - a card that says why, not a broken image."""
        img = np.full((360, 640, 3), (20, 14, 5), np.uint8)
        cv2.putText(img, "ML VIEW - WAITING FOR VIDEO", (40, 160),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (53, 107, 255), 2, cv2.LINE_AA)
        cv2.putText(img, self.source.status[:70], (40, 205),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 210, 225), 1, cv2.LINE_AA)
        cv2.putText(img, "model ready: %s (%s)" % (self.model_name, ", ".join(self.classes)),
                    (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 160, 180), 1, cv2.LINE_AA)
        return img

    def _publish(self, img, dets, fps, infer_ms, alert, size=None):
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self.cond:
            self.jpeg = jpg.tobytes()
            self.jpeg_seq += 1
            self.state = {
                "detections": dets,
                "fps": round(fps, 1) if fps is not None else 0.0,
                "infer_ms": round(infer_ms, 1) if infer_ms is not None else None,
                "frames": self.state["frames"] + (1 if size else 0),
                "alert": alert,
                "frame_size": size,
            }
            self.cond.notify_all()


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<title>SeaYou ML view</title><style>
body{margin:0;background:#050b14;color:#e6edf6;font:13px system-ui,sans-serif;
display:flex;flex-direction:column;align-items:center;gap:10px;padding:16px}
img{max-width:100%;border-radius:8px;border:1px solid #1d2a3b}
#d{font-variant-numeric:tabular-nums;color:#9fb2cc}</style></head><body>
<img src="/ml.mjpg" alt="ML view"><div id="d">connecting...</div>
<script>
setInterval(async()=>{try{const s=await (await fetch('/ml/status')).json();
document.getElementById('d').textContent=s.source_ok?
`${s.detections.length} detections - ${s.fps} fps - ${s.infer_ms} ms per frame`:
`waiting for video: ${s.source_status}`;}catch(e){}},1000);
</script></body></html>"""


def make_handler(det, src, source_label):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _cors(self):
            # The dashboard is served from the ground station's origin and
            # reads this from another port. Read-only data, so any origin.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(PAGE)
            elif path == "/ml/status":
                with det.lock:
                    body = dict(det.state)
                body.update({
                    "ok": True,
                    "source": source_label,
                    "source_ok": src.status == "ok" and body["frame_size"] is not None,
                    "source_status": src.status,
                    "model": det.model_name,
                    "classes": det.classes,
                    "conf_threshold": det.conf,
                    "alert_threshold": det.alert,
                })
                data = json.dumps(body).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            elif path == "/ml.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self._cors()
                self.end_headers()
                last = -1
                try:
                    while True:
                        with det.cond:
                            det.cond.wait_for(lambda: det.jpeg_seq != last, timeout=5.0)
                            jpg, last = det.jpeg, det.jpeg_seq
                        if jpg is None:
                            continue
                        self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode()
                                         + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            else:
                self.send_response(404)
                self._cors()
                self.end_headers()

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--source", default="drone",
                    help="'drone' (the ground station feed, default), a webcam index "
                         "like 0, a video file, a folder of images, or an RTSP/HTTP URL")
    ap.add_argument("--ground-station", default="http://127.0.0.1:8090",
                    help="where the ground station is, for --source drone")
    ap.add_argument("--port", type=int, default=8095)
    ap.add_argument("--model", default=None, help="path to a YOLO .pt (default: the team's best.pt)")
    ap.add_argument("--conf", type=float, default=DEFAULT_CONF,
                    help="lowest confidence drawn (default %.2f)" % DEFAULT_CONF)
    ap.add_argument("--alert", type=float, default=DEFAULT_ALERT,
                    help="Drowning confidence that turns the header red (default %.2f)" % DEFAULT_ALERT)
    ap.add_argument("--width", type=int, default=960,
                    help="downscale frames wider than this before inference")
    args = ap.parse_args()

    model = args.model or find_model()
    if not model or not os.path.isfile(model):
        raise SystemExit("Cannot find the model (best.pt). Pass --model PATH.")

    s = args.source
    if s == "drone":
        token = viewer_token()
        url = args.ground_station.rstrip("/") + "/stream.mjpg"
        if token:
            url += "?viewer=" + token
        src, label = MjpegSource(url, "the ground station"), "drone camera (via ground station)"
    elif s.isdigit():
        src, label = CaptureSource(int(s), loop=False), "webcam %s" % s
    elif os.path.isdir(s):
        src, label = FolderSource(s), "images in %s" % s
    elif s.startswith(("http://", "https://")) and "mjpg" in s:
        src, label = MjpegSource(s, s), s
    else:
        src, label = CaptureSource(s, loop=os.path.isfile(s)), s

    log.info("Loading %s ...", model)
    det = Detector(src, model, args.conf, args.alert, args.width)
    log.info("Model ready - classes: %s", ", ".join(det.classes))
    src.start()
    det.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(det, src, label))
    log.info("ML view on http://localhost:%d  (source: %s)", args.port, label)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
