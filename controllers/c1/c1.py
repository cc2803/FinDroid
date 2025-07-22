from controller import Robot, Motor, Lidar, InertialUnit, Camera, CameraRecognitionObject
import math
import random
import numpy as np
import cv2

class MazeNavigator:
    def __init__(self):
        self.robot = Robot()
        self.timestep = 64
        
        # Initialize motors
        self.wheels = [
            self.robot.getDevice("wheel1"),
            self.robot.getDevice("wheel2"),
            self.robot.getDevice("wheel3"),
            self.robot.getDevice("wheel4")
        ]
        for wheel in self.wheels:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
        
        # Initialize Lidar
        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.resolution = self.lidar.getHorizontalResolution()
        self.fov = self.lidar.getFov()
        
        # Initialize IMU
        self.imu = self.robot.getDevice("IMU")
        self.imu.enable(self.timestep)
        
        # Initialize Camera with recognition
        self.camera = self.robot.getDevice("CAM")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        
        # Mapping parameters
        self.GRID_SIZE = 100
        self.CELL_SCALE = 10
        self.grid = np.full((self.GRID_SIZE, self.GRID_SIZE), -1)
        self.center = self.GRID_SIZE // 2
        self.map_counter = 0
        
        # Navigation parameters
        self.speed = 0.05
        self.avoid_counter = 0
        self.turn_right = True
        self.OBSTACLE_THRESHOLD = 0.6
        self.MAX_RANGE = 5.0
        self.AVOID_DURATION = 20
        self.EXPLORE_DURATION = 30
        self.last_object_id = -1
        
        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def save_pgm(self, filename):
        with open(filename, 'w') as f:
            f.write(f"P2\n{self.GRID_SIZE} {self.GRID_SIZE}\n255\n")
            for row in self.grid:
                line = ' '.join(['0' if cell == 1 else '255' if cell == 0 else '128' for cell in row])
                f.write(line + '\n')

    def update_position(self):
        if self.avoid_counter == 0:
            rpy = self.imu.getRollPitchYaw()
            if rpy:
                self.theta = rpy[2]
                self.x += self.speed * math.cos(self.theta)
                self.y += self.speed * math.sin(self.theta)

    def process_lidar(self, ranges):
        obstacle_detected = False
        min_dist = self.MAX_RANGE
        
        # Check front sector
        for i in range(int(self.resolution*0.35), int(self.resolution*0.65)):
            if ranges[i] < self.OBSTACLE_THRESHOLD:
                obstacle_detected = True
                min_dist = min(min_dist, ranges[i])
        
        # Update occupancy grid
        for i in range(0, self.resolution, 2):
            d = ranges[i]
            if math.isinf(d) or d > self.MAX_RANGE:
                continue
            angle = -self.fov/2 + i*self.fov/self.resolution
            gx = self.x + d * math.cos(self.theta + angle)
            gy = self.y + d * math.sin(self.theta + angle)
            grid_x = int(self.center + gx * self.CELL_SCALE)
            grid_y = int(self.center - gy * self.CELL_SCALE)
            
            if 0 <= grid_x < self.GRID_SIZE and 0 <= grid_y < self.GRID_SIZE:
                self.grid[grid_y][grid_x] = 1
        
        return obstacle_detected, min_dist

    def process_objects(self):
        objects = self.camera.getRecognitionObjects()
        if objects:
            obj = objects[0]
            img_pos = obj.get_position_on_image()
            img_size = obj.get_size_on_image()
            print(f"Detected {obj.get_model()} at image position ({img_pos[0]:.0f}, {img_pos[1]:.0f})")
            
            if obj.get_id() != self.last_object_id:
                print(f"New object found! Exploring next area...")
                self.avoid_counter = self.EXPLORE_DURATION
                self.turn_right = random.choice([True, False])
                self.last_object_id = obj.get_id()
            return True
        return False

    def set_motor_speeds(self, left, right):
        for i in [0, 2]: self.wheels[i].setVelocity(left)
        for i in [1, 3]: self.wheels[i].setVelocity(right)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            ranges = self.lidar.getRangeImage()
            obstacle, min_dist = self.process_lidar(ranges)
            self.update_position()
            
            image = self.camera.getImageArray()

            if image is None:
                continue

            # Convert Webots image to OpenCV format
            img_np = np.array(image, dtype=np.uint8)
            img_np = np.reshape(img_np, (self.camera.getHeight(), self.camera.getWidth(), 4))
            img_rgb = cv2.cvtColor(img_np, cv2.COLOR_RGBA2RGB)

            # Draw recognition bounding boxes
            for i in range(self.camera.getRecognitionNumberOfObjects()):
                obj = self.camera.getRecognitionObjects()[i]
                pos = obj.getPositionOnImage()
                size = obj.getSizeOnImage()

                x, y = int(pos[0] - size[0] / 2), int(pos[1] - size[1] / 2)
                w, h = int(size[0]), int(size[1])

                cv2.rectangle(img_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img_rgb, obj.getModel(), (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Show image
            cv2.imshow("Camera", img_rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cv2.destroyAllWindows()           
            # Save map periodically
            self.map_counter += 1
            if self.map_counter % 40 == 0:
                pass
                #self.save_pgm(f"map_{self.map_counter//40}.pgm")
            
            # Object detection handling
            object_detected = self.process_objects()
            
            # Navigation logic
            if self.avoid_counter > 0:
                self.avoid_counter -= 1
                left = 3.0 if self.turn_right else -2.0
                right = -2.0 if self.turn_right else 3.0
            elif object_detected:
                pass  # Already handled by avoidance counter
            elif obstacle:
                self.avoid_counter = self.AVOID_DURATION
                self.turn_right = random.choice([True, False])
                print(f"Obstacle detected (d={min_dist:.2f}m), turning {'right' if self.turn_right else 'left'}")
                left = 3.0 if self.turn_right else -2.0
                right = -2.0 if self.turn_right else 3.0
            else:
                left = right = 3.0
            
            self.set_motor_speeds(left, right)

if __name__ == "__main__":
    controller = MazeNavigator()
    controller.run()
