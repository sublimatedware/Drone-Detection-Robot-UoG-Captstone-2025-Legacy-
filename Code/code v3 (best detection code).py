import cv2
import torch
import numpy as np
from ultralytics import YOLO

# device = 'cuda' # Uncomment to use GPU
device = 'cpu'

print(f"Using device: {device}")

# Load YOLOv8 model
model = YOLO('C:/VSCode/DroneDetection/Advanced-Aerial-Drone-Detection-System/best.pt').to(device)

# Set video source (webcam or video file)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

# Define the classes you want to detect
classes = ['Drone']

# Create a window for display
cv2.namedWindow('frame')

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    img = frame  # YOLOv8 can handle BGR directly

    # Run inference
    results = model(img)

    # Process the results and draw bounding boxes on the frame
    for result in results[0].boxes.data:  # Access detected objects
        x1, y1, x2, y2, conf, cls = result.tolist()
        if conf > 0.5 and classes[int(cls)] in classes:
            # Draw bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow('frame', frame)

    # Wait for key press to exit
    if cv2.waitKey(1) == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()