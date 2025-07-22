#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Lidar.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>

#include <Python.h>

#define TIME_STEP 64
#define GRID_SIZE 100
#define CELL_SCALE 10  // cells per meter
#define MAX_RANGE 5.0
#define OBSTACLE_THRESHOLD 0.6
#define AVOID_DURATION 20

using namespace webots;

void savePGM(const int grid[GRID_SIZE][GRID_SIZE], const std::string &filename) {
  std::ofstream out(filename);
  out << "P2\n" << GRID_SIZE << " " << GRID_SIZE << "\n255\n";
  for (int y = 0; y < GRID_SIZE; ++y) {
    for (int x = 0; x < GRID_SIZE; ++x) {
      if (grid[y][x] == 1)
        out << "0 ";     // black = obstacle
      else if (grid[y][x] == 0)
        out << "255 ";   // white = free
      else
        out << "128 ";   // gray = unknown
    }
    out << "\n";
  }
  out.close();
}

int main() {
  std::srand(std::time(nullptr));
  Robot *robot = new Robot();

  // Motors
  Motor *wheels[4];
  const char *wheelNames[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; ++i) {
    wheels[i] = robot->getMotor(wheelNames[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }

  // LiDAR
  Lidar *lidar = robot->getLidar("LIDAR");
  lidar->enable(TIME_STEP);
  lidar->enablePointCloud();
  int resolution = lidar->getHorizontalResolution();
  double fov = lidar->getFov();

  // IMU
  InertialUnit *imu = robot->getInertialUnit("IMU");
  if (!imu) {
    std::cerr << "IMU device not found!" << std::endl;
    delete robot;
    return 1;
  }
  imu->enable(TIME_STEP);

  // Camera (for object recognition)
  Camera *camera = robot->getCamera("CAM");
  camera->enable(TIME_STEP);
  camera->recognitionEnable(TIME_STEP);

  // Map grid initialization
  int grid[GRID_SIZE][GRID_SIZE];
  const int center = GRID_SIZE / 2;
  for (int y = 0; y < GRID_SIZE; ++y)
    for (int x = 0; x < GRID_SIZE; ++x)
      grid[y][x] = -1;  // unknown

  // Robot pose
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // yaw from IMU

  const double speed = 0.05;
  int mapCounter = 0;
  int avoidCounter = 0;
  bool turnRight = true;

  while (robot->step(TIME_STEP) != -1) {
    const float *ranges = lidar->getRangeImage();
    const double *rpy = imu->getRollPitchYaw();
    if (!rpy) {
      std::cerr << "Waiting for IMU data..." << std::endl;
      continue;
    }

    theta = rpy[2];

    // Obstacle detection (front-facing arc)
    bool obstacleAhead = false;
    double minDist = MAX_RANGE;
    for (int i = int(resolution * 0.35); i < int(resolution * 0.65); ++i) {
      float d = ranges[i];
      if (d < OBSTACLE_THRESHOLD) {
        obstacleAhead = true;
        if (d < minDist)
          minDist = d;
      }
    }

    // Update position
    if (avoidCounter == 0) {
      x += speed * cos(theta);
      y += speed * sin(theta);
    }

    // Update grid map with lidar points
    for (int i = 0; i < resolution; i += 2) {
      float d = ranges[i];
      if (d == INFINITY || d > MAX_RANGE) continue;

      float angle = -fov / 2.0 + i * (fov / resolution);
      float lx = d * cos(angle);
      float ly = d * sin(angle);
      float gx = x + lx * cos(theta) - ly * sin(theta);
      float gy = y + lx * sin(theta) + ly * cos(theta);

      int gridX = center + int(gx * CELL_SCALE);
      int gridY = center - int(gy * CELL_SCALE);

      if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE)
        grid[gridY][gridX] = 1;  // obstacle
    }

    // Save map occasionally
    if (++mapCounter % 40 == 0) {
      savePGM(grid, "map_" + std::to_string(mapCounter / 40) + ".pgm");
      std::cout << "Map saved\n";
    }

    // Movement logic
    double leftSpeed = 3.0;
    double rightSpeed = 3.0;

    if (avoidCounter > 0) {
      avoidCounter--;
      if (turnRight) {
        leftSpeed = 3.0;
        rightSpeed = -2.0;
      } else {
        leftSpeed = -2.0;
        rightSpeed = 3.0;
      }
    } else if (obstacleAhead) {
      avoidCounter = AVOID_DURATION;
      turnRight = std::rand() % 2;
      std::cout << "Obstacle ahead (dist=" << minDist << "). Turning " << (turnRight ? "right" : "left") << "...\n";
    }

    wheels[0]->setVelocity(leftSpeed);
    wheels[1]->setVelocity(rightSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);

    // Object recognition output
    int count = camera->getRecognitionNumberOfObjects();
    if (count > 0) {
      const CameraRecognitionObject *objects = camera->getRecognitionObjects();
      std::cout << "Detected " << count << " object(s):" << std::endl;
      for (int i = 0; i < count; ++i) {
        std::cout << "- ID: " << objects[i].id
                  << ", Position: (" << objects[i].position[0] << ", "
                  << objects[i].position[1] << ", "
                  << objects[i].position[2] << ")" << std::endl;
      }
    }
  }

  delete robot;
  return 0;
}