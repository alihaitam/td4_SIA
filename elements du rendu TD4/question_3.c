/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller for the Khepera III robot using the Braitenberg method.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define MAX_SENSOR_NUMBER 9
#define RANGE (2000)
#define BOUND(x, a, b) (((x) < (a)) ? (a) : ((x) > (b)) ? (b) : (x))

static WbDeviceTag sensors[MAX_SENSOR_NUMBER], left_motor, right_motor;
static double matrix[MAX_SENSOR_NUMBER][2];
static int num_sensors;
static double range;
static int time_step = 0;
static double max_speed = 0.0;
static double speed_unit = 1.0;

static void initialize() {
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();
  const char *robot_name = wb_robot_get_name();
  const char khepera_name[] = "ds0";

  char sensors_name[5];
  const double khepera3_matrix[9][2] = {{-5000, -5000},  {-20000, 40000}, {-30000, 50000}, {-70000, 70000}, {70000, -60000},
                                        {50000, -40000}, {40000, -20000}, {-5000, -5000},  {-10000, -10000}};

  if (strncmp(robot_name, "Khepera III", 11) == 0) {
    const double khepera3_max_speed = 19.1;
    const double khepera3_speed_unit = 0.00053429;

    num_sensors = 9;
    sprintf(sensors_name, "%s", khepera_name);
    range = RANGE;
    max_speed = khepera3_max_speed;
    speed_unit = khepera3_speed_unit;

    for (int i = 0; i < num_sensors; i++) {
      sensors[i] = wb_robot_get_device(sensors_name);
      wb_distance_sensor_enable(sensors[i], time_step);
      sensors_name[2]++;

      for (int j = 0; j < 2; j++)
        matrix[i][j] = khepera3_matrix[i][j];
    }
  } else {
    fprintf(stderr, "This controller is for the Khepera III robot only.\n");
    exit(EXIT_FAILURE);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  printf("The Khepera III robot is initialized with %d distance sensors.\n", num_sensors);
}

int main() {
  initialize();

  while (wb_robot_step(time_step) != -1) {
    double speed[2] = {0.0, 0.0};
    double sensors_value[MAX_SENSOR_NUMBER];

    for (int i = 0; i < num_sensors; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(sensors[i]);
    }

    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < num_sensors; j++) {
        speed[i] += speed_unit * matrix[j][i] * (1.0 - (sensors_value[j] / range));
      }
      speed[i] = BOUND(speed[i], -max_speed, max_speed);
    }

    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  return 0;
}