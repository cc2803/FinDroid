from controller import Robot, Camera
import numpy as np
import cv2
import requests,time

TIME_STEP = 64
robot = Robot()

FLASK_SERVER_URL = "http://localhost:5000/update"

ds = []
dsNames = ['ds_right', 'ds_left']
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(TIME_STEP)

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

avoidObstacleCounter = 0

camera = robot.getDevice("CAM")
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    
    leftSpeed = 1.0
    rightSpeed = 1.0
    
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    
    else:  # read sensors
        for i in range(2):
            if ds[i].getValue() < 950.0:
                avoidObstacleCounter = 100
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

    # Camera frame processing
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    np_image = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    bgr_image = cv2.cvtColor(np_image, cv2.COLOR_BGRA2BGR)

    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')  # Generate timestamp

    objects = camera.getRecognitionObjects()
    data = []
    for i, obj in enumerate(objects):
        model = obj.getModel()
        prec = "YES" if model in ["MobilePhone", "Wallet", "Jewelry", "Keys"] else "NO"
        entry = {
            "id": i + 1,
            "name": model,
            "object_id": obj.getId(),
            "timestamp": timestamp,
            "precious": prec
        }
        data.append(entry)
    try:
        requests.post(FLASK_SERVER_URL, json={"objects": data})
    except:
        print("Flask server not reachable")

    # Object recognition
    objects = camera.getRecognitionObjects()
    if objects:
        print("Detected objects:")
        for obj in objects:
            print(f" - {obj.getModel()} at {obj.getPosition()}")

            # Optional: Draw bounding box on image
            pos = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            x = int(pos[0] - size[0]/2)
            y = int(pos[1] - size[1]/2)
            w = int(size[0])
            h = int(size[1])
            cv2.rectangle(bgr_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Optional: Show camera feed
    cv2.imshow("CAM View", bgr_image)
    if cv2.waitKey(1) == 27:  # ESC to close
        break

cv2.destroyAllWindows()