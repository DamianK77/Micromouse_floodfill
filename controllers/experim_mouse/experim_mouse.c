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
#define DISTANCE_THRESHOLD 300
#define MAX_SPEED 3.0
#define TIME_WAIT 0.5

WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
double expected_dl = 0.0;
double expected_dr = 0.0;
int orient = 0; // 0 = up 1 = right 2 = down 3 = left
int xy_pos[2] = {0, 0};

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
  orient = (orient + 1) % 4;
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
  orient = (orient + 3) % 4;
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

  int i;
  double sensors_value[8];
  bool wall_up = false;
  bool wall_right = false;
  bool wall_down = false;
  bool wall_left = false;
  int time_step;
  int camera_time_step;

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

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    passive_wait(TIME_WAIT); // wait for sensors to get stable values
    printf("------NEXT MOVE------\n");

    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }

      /* DATA ANALYSIS*/

    //determine if there is a wall in each direction
    wall_down = sensors_value[4] > DISTANCE_THRESHOLD;
    wall_left = sensors_value[5] > DISTANCE_THRESHOLD;
    wall_up = sensors_value[7] > DISTANCE_THRESHOLD;
    wall_right = sensors_value[2] > DISTANCE_THRESHOLD;

    //determine which type of intersection it is assuming orientatnion 0 (up)
    int curr_intersection = 0;
    if (wall_down && !wall_left && !wall_up && !wall_right) {
      curr_intersection = 0;
    } else if (!wall_down && wall_left && !wall_up && !wall_right) {
      curr_intersection = 1;
    } else if (!wall_down && !wall_left && wall_up && !wall_right) {
      curr_intersection = 2;
    } else if (!wall_down && !wall_left && !wall_up && wall_right) {
      curr_intersection = 3;
    } else if (wall_down && wall_left && !wall_up && !wall_right) {
      curr_intersection = 4;
    } else if (!wall_down && wall_left && wall_up && !wall_right) {
      curr_intersection = 5;
    } else if (!wall_down && !wall_left && wall_up && wall_right) {
      curr_intersection = 6;
    } else if (wall_down && !wall_left && !wall_up && wall_right) {
      curr_intersection = 7;
    } else if (wall_down && wall_left && !wall_up && wall_right) {
      curr_intersection = 8;
    } else if (wall_down && wall_left && wall_up && !wall_right) {
      curr_intersection = 9;
    } else if (!wall_down && wall_left && wall_up && wall_right) {
      curr_intersection = 10;
    } else if (wall_down && !wall_left && wall_up && wall_right) {
      curr_intersection = 11;
    } else if (!wall_down && wall_left && !wall_up && wall_right) {
      curr_intersection = 12;
    } else if (wall_down && !wall_left && wall_up && !wall_right) {
      curr_intersection = 13;
    }

    //correct intersection type based on orientation
    int curr_intersection_abs;
    if (curr_intersection >= 0 && curr_intersection < 4) {
      curr_intersection_abs = (curr_intersection + orient) % 4;
    } else if (curr_intersection >= 4 && curr_intersection < 8) {
      curr_intersection_abs = ((curr_intersection + orient) % 4) + 4;
    } else if (curr_intersection >= 8 && curr_intersection < 12) {
      curr_intersection_abs = ((curr_intersection + orient) % 4) + 8;
    } else if (curr_intersection == 12 && (orient == 1 || orient == 3)) {
      curr_intersection_abs = 13;
    } else if (curr_intersection == 13 && (orient == 1 || orient == 3)) {
      curr_intersection_abs = 12;
    } else {
      curr_intersection_abs = curr_intersection;
    }


    printf("Orientation: ");
    if (orient == 0) {
      printf("UP\n");
    } else if (orient == 1) {
      printf("RIGHT\n");
    } else if (orient == 2) {
      printf("DOWN\n");
    } else if (orient == 3) {
      printf("LEFT\n");
    }

    printf("Current intersection: %i\n", curr_intersection_abs);

    /* set speed values */

    //calibrate distance if front wall available
    if (curr_intersection == 2 || curr_intersection == 5 || curr_intersection == 6 || curr_intersection == 9 || curr_intersection == 10 || curr_intersection == 11 || curr_intersection == 13) {
      if (sensors_value[7] < 480 && sensors_value[7] > DISTANCE_THRESHOLD) {
        double tmp_val;
        do {
          wb_motor_set_velocity(left_motor, 0.5);
          wb_motor_set_velocity(right_motor, 0.5);
          tmp_val = wb_distance_sensor_get_value(distance_sensor[7]);
          step();
        } while (tmp_val < 480);
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
        expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
      }

      if (sensors_value[7] > 520) {
        double tmp_val;
        do {
          wb_motor_set_velocity(left_motor, -0.5);
          wb_motor_set_velocity(right_motor, -0.5);
          tmp_val = wb_distance_sensor_get_value(distance_sensor[7]);
          step();
        } while (tmp_val > 520);
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
        expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
      }
      //end of calibration
    }

    //decision made based on intersection type

    if (curr_intersection == 0 || curr_intersection == 2 || curr_intersection == 3|| curr_intersection == 6 || curr_intersection == 7 || curr_intersection == 11 || curr_intersection == 13) {
      //if left turn is available, take it
      turn_left();
      passive_wait(TIME_WAIT);
      move_forward();
    } else if (curr_intersection == 1 || curr_intersection == 4 || curr_intersection == 8 || curr_intersection == 12) {
      //if left turn is not available but can go straight, go straight
      move_forward();
    } else if (curr_intersection == 5 || curr_intersection == 9) {
      //if left turn is not available and cannot go straight but can go right, turn right
      turn_right();
      passive_wait(TIME_WAIT);
      move_forward();
    } else if (curr_intersection == 10) {
      //if left up and right not available, turn around
      turn_right();
      passive_wait(TIME_WAIT);
      turn_right();
    }

    /* OLD ALGORITHM */

    // if (!(sensors_value[5] > DISTANCE_THRESHOLD)) {    //detect availble left turn
    //   turn_left();
    //   passive_wait(TIME_WAIT);
    //   move_forward();
    // } else if (sensors_value[7] > DISTANCE_THRESHOLD) {     //detect wall if no left turn, turn 180deg if wall is seen
    //   //calibrate if a wall is seen
    //   if (sensors_value[7] < 480 && sensors_value[7] > DISTANCE_THRESHOLD) {
    //     double tmp_val;
    //     do {
    //       wb_motor_set_velocity(left_motor, 0.5);
    //       wb_motor_set_velocity(right_motor, 0.5);
    //       tmp_val = wb_distance_sensor_get_value(distance_sensor[7]);
    //       step();
    //     } while (tmp_val < 480);
    //     wb_motor_set_velocity(left_motor, 0.0);
    //     wb_motor_set_velocity(right_motor, 0.0);
    //     expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
    //     expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
    //   }

    //   if (sensors_value[7] > 520) {
    //     double tmp_val;
    //     do {
    //       wb_motor_set_velocity(left_motor, -0.5);
    //       wb_motor_set_velocity(right_motor, -0.5);
    //       tmp_val = wb_distance_sensor_get_value(distance_sensor[7]);
    //       step();
    //     } while (tmp_val > 520);
    //     wb_motor_set_velocity(left_motor, 0.0);
    //     wb_motor_set_velocity(right_motor, 0.0);
    //     expected_dl = wb_position_sensor_get_value(left_position_sensor) * WHEEL_RADIUS;
    //     expected_dr = wb_position_sensor_get_value(right_position_sensor) * WHEEL_RADIUS;
    //   }
    //   //end of calibration
      
    //   turn_right();
    //   passive_wait(TIME_WAIT);
    //   turn_right();

    // } else {    //move forward if no left turn or wall is seen
    //   move_forward();  

    // }
    


    passive_wait(TIME_WAIT);

  }

  wb_robot_cleanup();

  return 0;
}
