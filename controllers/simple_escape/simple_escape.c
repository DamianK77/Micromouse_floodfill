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

#include <stdio.h>
#include <string.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)
#define TIME_90deg 2.23
#define TIME_grid 5.0
#define DISTANCE_THRESHOLD 75
#define MAX_SPEED 3.0
#define TIME_WAIT 0.5

WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
double expected_dl = 0.0;
double expected_dr = 0.0;

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void turn_right() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
      
  passive_wait(TIME_WAIT);

  wb_motor_set_velocity(left_motor, 1);
  wb_motor_set_velocity(right_motor, -1);

  passive_wait(TIME_90deg);

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
  expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
}

static void turn_left() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
      
  passive_wait(TIME_WAIT);

  wb_motor_set_velocity(left_motor, -1);
  wb_motor_set_velocity(right_motor, 1);

  passive_wait(TIME_90deg);

  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
  expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
}

static void move_forward() {
    expected_dl = expected_dl + 0.1;
    expected_dr = expected_dr + 0.1;
    double curr_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
    while (curr_dl <= expected_dl) {
      wb_motor_set_velocity(left_motor, MAX_SPEED);
      wb_motor_set_velocity(right_motor, MAX_SPEED);
      curr_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
      step();
    }
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
    expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
    expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
}

int main(int argc, char *argv[]) {
  /* define variables */

  int i, j;
  double speed[2];
  double sensors_value[8];
  int time_step;
  int camera_time_step;


  bool calibrated_y, calibrated_x = false;

  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 8;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 8;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];

    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }

  //set acceleration
  wb_motor_set_acceleration(left_motor, 100);
  wb_motor_set_acceleration(right_motor, 100);

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    passive_wait(TIME_WAIT); // wait for sensors to get stable values

    printf("Sensors taken\n");

    for (i = 0; i < 8; i++)
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);

    const double *a = wb_accelerometer_get_values(accelerometer);
    double l = wb_position_sensor_get_value(left_position_sensor);
    double r = wb_position_sensor_get_value(right_position_sensor);
    double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
    double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
    double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
    double curr_dl = dl;
    double curr_dr = dr;
    
    //printf("accelerometer values = %0.2f %0.2f %0.2f\n", a[0], a[1], a[2]);

    /* compute odometry and speed values*/
    //compute_odometry(left_position_sensor, right_position_sensor);

    /* set speed values */
    if (sensors_value[7] > DISTANCE_THRESHOLD) {
      turn_left();
    } else {
      move_forward();
    }

    passive_wait(TIME_WAIT);
    
    // printf("expected_dl = %0.5f\n", expected_dl);

    // //turn right
    // while (curr_dl <= expected_dl || curr_dr >= expected_dr) {
    //   if (curr_dl <= expected_dl) {
    //     wb_motor_set_velocity(left_motor, 0.3*MAX_SPEED);
    //   } else {
    //     wb_motor_set_velocity(left_motor, 0.0);
    //   }
    //   if (curr_dr >= expected_dr) {
    //     wb_motor_set_velocity(right_motor, -0.3*MAX_SPEED);
    //   } else {
    //     wb_motor_set_velocity(right_motor, 0.0);
    //   }
    //   curr_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
    //   curr_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
    //   step();
    // }

    // wb_motor_set_velocity(left_motor, 0.0);
    // wb_motor_set_velocity(right_motor, 0.0);

    // passive_wait(3.0);



    // passive_wait(3.0);

    // wb_motor_set_velocity(left_motor, 0.0);
    // wb_motor_set_velocity(right_motor, 0.0);

    //printf("wheel sensors distance = %0.2f %0.2f\n", dl, dr);

  }

  wb_robot_cleanup();

  return 0;
}
