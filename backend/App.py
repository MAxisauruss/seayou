from ultralytics import YOLO
import cv2
import os

## Load the YOLO model
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "runs", "detect", "train", "weights", "best.pt")
model = YOLO(MODEL_PATH)

## Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not open camera. Try a different index (0, 1, 2) or check permissions.")
    exit()

print("Camera opened. Press 'q' to quit.")

while True:
    ok, frame = cap.read()
    if not ok:
        print("Failed to read frame from camera.")
        break

    # Run inference on this frame
    results = model.predict(frame, verbose=False)[0]

    # Collect and print detections
    detections = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        label = results.names[cls_id]
        detections.append({"label": label, "confidence": round(conf, 2)})

    if detections:
        print(detections)

    # Draw boxes on the frame and show it live
    annotated = results.plot()
    cv2.imshow("Drowning Detection", annotated)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()