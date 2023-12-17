#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 10.1
#define NUM_SENSORS 9
#define OBSTACLE_THRESHOLD 50

WbDeviceTag left_motor, right_motor, sensors[NUM_SENSORS];

void initialize() {
  wb_robot_init();
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  char sensor_name[4];
  for (int i = 0; i < NUM_SENSORS; ++i) {
    sprintf(sensor_name, "ds%d", i);
    sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(sensors[i], TIME_STEP);
  }
}

bool detect_obstacle() {
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if (wb_distance_sensor_get_value(sensors[i]) > OBSTACLE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

int main() {
  initialize();

  while (wb_robot_step(TIME_STEP) != -1) {
    if (detect_obstacle()) {
      // Stop the robot
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    } else {
      // Move straight forward
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, MAX_SPEED);
    }
  }
  
  wb_robot_cleanup();
  return 0;
}
