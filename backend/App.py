from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import os
from datetime import datetime

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "runs", "detect", "train", "weights", "best.pt")
model = YOLO(MODEL_PATH)

CONFIDENCE_THRESHOLD = 0.90
TARGET_LABEL = "drowning"


def build_alert(confidence, timestamp):
    return {
        "type": "drowning_detected",
        "confidence": round(confidence, 4),
        "timestamp": timestamp,
    }


def detect_drowning(resolution=(640, 480)):

    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": resolution, "format": "RGB888"})
    picam2.configure(config)
    picam2.start()

    print("Pi camera started.")

    try:
        while True:
            frame = picam2.capture_array()  # RGB888 numpy array

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
        picam2.stop()


#This is code for testing the camera with the drone.
if __name__ == "__main__":
    for frame, alert in detect_drowning():
        cv2.imshow("Drowning Detection", frame)
        # Alert handling logic can be added here, e.g., sending the alert to a server or logging it. Or a flashing LED on the pi
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()