from ultralytics import YOLO
import cv2
import os
from datetime import datetime

# Loading the YOLO model from the specified path
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "runs", "detect", "train", "weights", "best.pt")
model = YOLO(MODEL_PATH)

CONFIDENCE_THRESHOLD = 0.90
TARGET_LABEL = "drowning"    # confirm that this matches the label used in your YOLO training dataset

# build_alert function creates a dictionary containing the alert information when a drowning detection occurs.
def build_alert(confidence, timestamp):
    return {
        "type": "drowning_detected",
        "confidence": round(confidence, 4),
        "timestamp": timestamp,
    }

# detect_drowning function captures frames from a camera source (like an RTSP stream) and processes them to detect drowning events.
def detect_drowning(source):
    cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"Could not open camera feed: {source}")
        return

    print(f"Camera feed opened: {source}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame — feed ended or connection lost.")
                break

            results = model.predict(frame, verbose=False)[0]

            alert = None
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = results.names[cls_id]

                if label == TARGET_LABEL and conf >= CONFIDENCE_THRESHOLD:
                    alert = build_alert(conf, datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
                    print(f"ALERT: {alert}")
                    break  # This is one alert per frame; if we want multiple alerts per frame, we can remove this break.

            annotated_frame = results.plot()

            yield annotated_frame, alert

    finally:
        cap.release()


# Test the camera feed and detection functionality when the script is run directly.
if __name__ == "__main__":
    CAMERA_SOURCE = "rtsp://user:password@192.168.1.50:554/stream1"  # replace with your actual camera URL/index

    for frame, alert in detect_drowning(CAMERA_SOURCE):
        cv2.imshow("Beach Drowning Detection", frame)
        # Alert handling logic can be added here, e.g., sending the alert to a server or logging it.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()