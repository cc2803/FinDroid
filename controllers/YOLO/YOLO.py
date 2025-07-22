from controller import Robot, Camera, DistanceSensor, Motor, Lidar, InertialUnit
import numpy as np
import cv2
from ultralytics import YOLO
import os

# Constants
TIME_STEP = 64
MODEL_PATH = "E:/WebotSims/controllers/YOLO/best.pt"

# --- Robot Setup ---
robot = Robot()

# Motors
wheels = [robot.getDevice(f'wheel{i+1}') for i in range(4)]
for w in wheels:
    w.setPosition(float('inf'))
    w.setVelocity(0.0)

# Camera setup
camera = robot.getDevice("CAM")
camera.enable(TIME_STEP)

# Distance sensors
ds_names = ['ds_right', 'ds_left']
ds = [robot.getDevice(name) for name in ds_names]
for sensor in ds:
    sensor.enable(TIME_STEP)

# Optional: Lidar and IMU setup (if you use them)
lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

imu = robot.getDevice('IMU')
imu.enable(TIME_STEP)

# Load YOLOv5 model
model = YOLO(MODEL_PATH)

print("[INFO] YOLO model loaded successfully.")

# Main loop
avoid_counter = 0

while robot.step(TIME_STEP) != -1:
    # Basic obstacle avoidance
    left_speed = 3.0
    right_speed = 3.0

    if avoid_counter > 0:
        avoid_counter -= 1
        left_speed = 3.0
        right_speed = -3.0
    else:
        for i in range(2):
            if ds[i].getValue() < 950:
                avoid_counter = 25

    # Set wheel speeds
    for i in (0, 2):
        wheels[i].setVelocity(left_speed)
    for i in (1, 3):
        wheels[i].setVelocity(right_speed)

    # Camera capture and YOLO inference
    image = camera.getImage()
    if image:
        width = camera.getWidth()
        height = camera.getHeight()
        np_image = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        bgr_image = cv2.cvtColor(np_image, cv2.COLOR_BGRA2BGR)

        # Inference
        results = model(bgr_image)[0]

        # Draw and log detected objects
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = f"{model.names[cls_id]} {box.conf[0]:.2f}"

            cv2.rectangle(bgr_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(bgr_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            print(f"[DETECTED] {label} at [{x1},{y1},{x2},{y2}]")

        # Show detection result (optional)
        cv2.imshow("YOLOv5 Detections", bgr_image)
        if cv2.waitKey(1) == 27:  # ESC
            break

cv2.destroyAllWindows()
