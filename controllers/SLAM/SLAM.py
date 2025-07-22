from controller import Robot, Motor, Lidar, InertialUnit, Camera
import math
import random
import time
import os

# Constants
TIME_STEP = 64
GRID_SIZE = 100
CELL_SCALE = 10  # cells per meter
MAX_RANGE = 5.0
OBSTACLE_THRESHOLD = 0.6
AVOID_DURATION = 20

def save_pgm(grid, filename):
    # Create maps directory if it doesn't exist
    folder = 'maps'
    os.makedirs(folder, exist_ok=True)  # This line was added
    
    filepath = os.path.join(folder, filename)  # This line was modified
    with open(filepath, 'w') as out:
        out.write(f"P2\n{len(grid)} {len(grid)}\n255\n")
        for row in grid:
            out.write(' '.join(map(str, [
                0 if cell == 1 else 255 if cell == 0 else 128 
                for cell in row
            ])) + '\n')

def main():
    robot = Robot()
    random.seed(time.time())

    # Initialize motors
    wheels = [
        robot.getDevice('wheel1'),
        robot.getDevice('wheel2'),
        robot.getDevice('wheel3'),
        robot.getDevice('wheel4')
    ]
    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(0.0)

    # Initialize Lidar
    lidar = robot.getDevice('lidar')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()
    resolution = lidar.getHorizontalResolution()
    fov = lidar.getFov()

    # Initialize IMU
    imu = robot.getDevice('IMU')
    imu.enable(TIME_STEP)

    # Initialize Camera
    camera = robot.getDevice('CAM')
    camera.enable(TIME_STEP)
    camera.recognitionEnable(TIME_STEP)

    # Initialize grid map
    grid = [[-1 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    center = GRID_SIZE // 2

    # Robot state
    x, y, theta = 0.0, 0.0, 0.0
    speed = 0.05
    map_counter = 0
    avoid_counter = 0
    turn_right = True

    while robot.step(TIME_STEP) != -1:
        ranges = lidar.getRangeImage()
        rpy = imu.getRollPitchYaw()
        theta = rpy[2]

        # Obstacle detection
        obstacle_ahead = False
        min_dist = MAX_RANGE
        start_idx = int(resolution * 0.35)
        end_idx = int(resolution * 0.65)
        
        for i in range(start_idx, end_idx):
            d = ranges[i]
            if d < OBSTACLE_THRESHOLD:
                obstacle_ahead = True
                min_dist = min(min_dist, d)

        # Update position
        if avoid_counter == 0:
            x += speed * math.cos(theta)
            y += speed * math.sin(theta)

        # Update grid map
        for i in range(0, resolution, 2):
            d = ranges[i]
            if math.isinf(d) or d > MAX_RANGE:
                continue

            angle = -fov/2 + i * (fov/resolution)
            lx = d * math.cos(angle)
            ly = d * math.sin(angle)
            gx = x + lx * math.cos(theta) - ly * math.sin(theta)
            gy = y + lx * math.sin(theta) + ly * math.cos(theta)

            grid_x = center + int(gx * CELL_SCALE)
            grid_y = center - int(gy * CELL_SCALE)

            if 0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE:
                grid[grid_y][grid_x] = 1

        # Save map
        map_counter += 1
        if map_counter % 40 == 0:
            save_pgm(grid, f"map_{map_counter//40}.pgm")  # Now saves in maps/ folder
            print("Map saved in maps/ directory")

        # Movement logic
        left_speed = 3.0
        right_speed = 3.0

        if avoid_counter > 0:
            avoid_counter -= 1
            if turn_right:
                left_speed, right_speed = 3.0, -2.0
            else:
                left_speed, right_speed = -2.0, 3.0
        elif obstacle_ahead:
            avoid_counter = AVOID_DURATION
            turn_right = random.choice([True, False])
            print(f"Obstacle ahead (dist={min_dist:.2f}). Turning {'right' if turn_right else 'left'}...")

        # Set motor speeds
        wheels[0].setVelocity(left_speed)
        wheels[1].setVelocity(right_speed)
        wheels[2].setVelocity(left_speed)
        wheels[3].setVelocity(right_speed)

        # Object recognition
        objects = camera.getRecognitionObjects()
        if objects:
            print(f"Detected {len(objects)} object(s):")
            for obj in objects:
                pos = obj.getPosition()
                print(f"- ID: {obj.getId()}, Position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

if __name__ == "__main__":
    main()
