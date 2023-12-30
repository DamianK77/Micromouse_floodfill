/**
 * @file mouse_floodfill.c
 * @author Damian Kąkol
 * @brief MICROMOUSE FLOODFILL ALGORITHM WEBOTS
 * 
 * @copyright Apache 2.0
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
int xy_pos[2] = {0, 0}; // start position
const int xy_pos_start[2] = {0, 0}; // start position
const int target_xy_pos[2] = {4, 9}; // target position
int intersection_map[10][10]; // map of intersections
int flood_array[10][10]; // array for floodfill
bool mapped = false; // true if maze has been mapped
bool pathed = false; // true if path has been established
int path[100]; // array of directions to take to reach target
int path_length = 0; // length of path

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

//function to drive in a absolute direction (based on orientation) using turn_left, turn_right and move_forward
void drive_absolute(int direction) {
  printf("Driving in direction %i\n", direction);
  printf("Current orientation: %i\n", orient);
  direction = abs((direction - orient + 4) % 4);
  printf("Driving in corrected direction %i\n", direction);
  if (direction == 0) {
    move_forward();
  } else if (direction == 1) {
    turn_right();
    move_forward();
  } else if (direction == 2) {
    turn_right();
    turn_right();
    move_forward();
  } else if (direction == 3) {
    turn_left();
    move_forward();
  }
  passive_wait(TIME_WAIT);
}

// Function to update floodfill array based on the current intersection map
void updateFloodfillArray() {
  int queue[100][2]; // queue of coordinates to process for next floodfill iteration
  //fill array with -1s
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++){
      flood_array[i][j] = -1;
    }
  }
  
  //set target cell to 0
  flood_array[target_xy_pos[0]][target_xy_pos[1]] = 0;


  //initialize queue with target cell
  int queue_size = 1;
  queue[0][0] = target_xy_pos[0];
  queue[0][1] = target_xy_pos[1];

  //while queue is not empty
  while (queue_size > 0) {
    //get coordinates of first cell in queue
    int curr_x = queue[0][0];
    int curr_y = queue[0][1];

    //remove first cell from queue
    for (int i = 0; i < queue_size - 1; i++) {
      queue[i][0] = queue[i + 1][0];
      queue[i][1] = queue[i + 1][1];
    }
    queue_size--;

    //get value of current cell
    int curr_val = flood_array[curr_x][curr_y];
    int curr_inter = intersection_map[curr_x][curr_y];

    //if cell is not a wall and has not been visited yet (value is -1) then update value to curr_val + 1 and add all neighbouring cells to queue
    // for up cell, if current cell has no wall up and up cell has no wall down etc.
    int up_inter = intersection_map[curr_x][curr_y + 1];
    int down_inter = intersection_map[curr_x][curr_y - 1];
    int left_inter = intersection_map[curr_x - 1][curr_y];
    int right_inter = intersection_map[curr_x + 1][curr_y];

    //get flood array vals
    int up_val = flood_array[curr_x][curr_y + 1];
    int down_val = flood_array[curr_x][curr_y - 1];
    int left_val = flood_array[curr_x - 1][curr_y];
    int right_val = flood_array[curr_x + 1][curr_y];

    //for up cell
    if (curr_y < 9 && (up_val > curr_val + 1 || up_val == -1) && (curr_inter == 0 || curr_inter == 1 || curr_inter == 3 || curr_inter == 4 || curr_inter == 7 || curr_inter == 8 || curr_inter == 12 || curr_inter == -1) && (up_inter == 1 || up_inter == 2 || up_inter == 3 || up_inter == 5 || up_inter == 6 || up_inter == 10 || up_inter == 12 || up_inter == -1)) {
      flood_array[curr_x][curr_y + 1] = curr_val + 1;
      queue[queue_size][0] = curr_x;
      queue[queue_size][1] = curr_y + 1;
      queue_size++;
    }

    //for down cell
    if (curr_y > 0 && (down_val > curr_val + 1 || down_val == -1) && (curr_inter == 1 || curr_inter == 2 || curr_inter == 3 || curr_inter == 5 || curr_inter == 6 || curr_inter == 10 || curr_inter == 12 || curr_inter == -1) && (down_inter == 0 || down_inter == 1 || down_inter == 3 || down_inter == 4 || down_inter == 7 || down_inter == 8 || down_inter == 12 || down_inter == -1)) {
      flood_array[curr_x][curr_y - 1] = curr_val + 1;
      queue[queue_size][0] = curr_x;
      queue[queue_size][1] = curr_y - 1;
      queue_size++;
    }

    //for left cell
    if (curr_x > 0 && (left_val > curr_val + 1 || left_val == -1) && (curr_inter == 0 || curr_inter == 2 || curr_inter == 3 || curr_inter == 6 || curr_inter == 7 || curr_inter == 11 || curr_inter == 13 || curr_inter == -1) && (left_inter == 0 || left_inter == 1 || left_inter == 2 || left_inter == 4 || left_inter == 5 || left_inter == 9 || left_inter == 13 || left_inter == -1)) {
      flood_array[curr_x - 1][curr_y] = curr_val + 1;
      queue[queue_size][0] = curr_x - 1;
      queue[queue_size][1] = curr_y;
      queue_size++;
    }

    //for right cell
    if (curr_x < 9 && (right_val > curr_val + 1 || right_val == -1) && (curr_inter == 0 || curr_inter == 1 || curr_inter == 2 || curr_inter == 4 || curr_inter == 5 || curr_inter == 9 || curr_inter == 13 || curr_inter == -1) && (right_inter == 0 || right_inter == 2 || right_inter == 3 || right_inter == 6 || right_inter == 7 || right_inter == 11 || right_inter == 13 || right_inter == -1)) {
      flood_array[curr_x + 1][curr_y] = curr_val + 1;
      queue[queue_size][0] = curr_x + 1;
      queue[queue_size][1] = curr_y;
      queue_size++;
    }

  }

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

  //fill map with -1s
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++){
      intersection_map[i][j] = -1;
    }
  }

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

    //loop for maping
    //=======================MAPING=========================
    while (!mapped) {
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

    //update intersection map

    intersection_map[xy_pos[0]][xy_pos[1]] = curr_intersection_abs;

    //regenerate floodfill array based on new intersection map by starting from the target and updating the value of each neighbour cell to one more than itself (while respecting walls)

    updateFloodfillArray();


    //print data
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

    printf("Current position: (%i, %i)\n", xy_pos[0], xy_pos[1]);

    //print intersection map
    printf("Intersection map:\n");
    for (int i = 9; i >= 0; i--) {
      for (int j = 0; j < 10; j++) {
        printf("%i ", intersection_map[j][i]);
      }
      printf("\n");
    }

    //print floodfill array
    printf("Floodfill array:\n");
    for (int i = 9; i >= 0; i--) {
      for (int j = 0; j < 10; j++) {
        printf("%i ", flood_array[j][i]);
      }
      printf("\n");
    }

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
      move_forward();
    }

    //update xy position
    if (orient == 0) {
      xy_pos[1] = xy_pos[1] + 1;
    } else if (orient == 1) {
      xy_pos[0] = xy_pos[0] + 1;
    } else if (orient == 2) {
      xy_pos[1] = xy_pos[1] - 1;
    } else if (orient == 3) {
      xy_pos[0] = xy_pos[0] - 1;
    }
    
    //end condition
    if (xy_pos[0] == target_xy_pos[0] && xy_pos[1] == target_xy_pos[1]) {
      printf("Target reached!\n");
      mapped = true;
    }
    
    passive_wait(TIME_WAIT);

  }

  //===============PATHING================

  if (mapped && !pathed) {
    //loop for pathing
    int curr_x = xy_pos_start[0];
    int curr_y = xy_pos_start[1];

    while (flood_array[curr_x][curr_y] != 0) { //until we are at the end of the maze
      //get current flood value and intersection type
      int curr_flood = flood_array[curr_x][curr_y];
      int curr_inter = intersection_map[curr_x][curr_y];
      //get flood values of neighbouring cells
      //if a cell has a flood value one less than the current cell, move to that cell and note direction in path array
      //also check if you are able to move in that direction (no wall)
      if (curr_y < 9 && flood_array[curr_x][curr_y + 1] == curr_flood - 1 && (curr_inter == 0 || curr_inter == 1 || curr_inter == 3 || curr_inter == 4 || curr_inter == 7 || curr_inter == 8 || curr_inter == 12)) {
        curr_y++; //up
        path[path_length] = 0;
        path_length++;
      } else if (curr_y > 0 && flood_array[curr_x][curr_y - 1] == curr_flood - 1 && (curr_inter == 1 || curr_inter == 2 || curr_inter == 3 || curr_inter == 5 || curr_inter == 6 || curr_inter == 10 || curr_inter == 12)) {
        curr_y--; //down
        path[path_length] = 2;
        path_length++;
      } else if (curr_x > 0 && flood_array[curr_x - 1][curr_y] == curr_flood - 1 && (curr_inter == 0 || curr_inter == 2 || curr_inter == 3 || curr_inter == 6 || curr_inter == 7 || curr_inter == 11 || curr_inter == 13)) {
        curr_x--; //left
        path[path_length] = 3;
        path_length++;
      } else if (curr_x < 9 && flood_array[curr_x + 1][curr_y] == curr_flood - 1 && (curr_inter == 0 || curr_inter == 1 || curr_inter == 2 || curr_inter == 4 || curr_inter == 5 || curr_inter == 9 || curr_inter == 13)) {
        curr_x++; //right
        path[path_length] = 1;
        path_length++;
      }
    }
    pathed = true;
    printf("Path found!\n");
    printf("Path length: %i\n", path_length);
    printf("Path: ");
    for (int i = 0; i < path_length; i++) {
      printf("%i ", path[i]);
    }
    printf("\n");
  }

  //===============FOLLOW PATH BACK================
  //drive back to starting position
  if (pathed) {
    for (int i = path_length - 1; i >= 0; i--) {
      int path_back = abs((path[i] - 2 + 4) % 4); //invert movements
      drive_absolute(path_back);
    }
  }

  //===============FOLLOW SHORTEST ROUTE================
  if (pathed) {
    //drive to target
    for (int i = 0; i < path_length; i++) {
      drive_absolute(path[i]);
    }
    while (true) {// fall into infinite loop after accomplishing the task
      step();
    }
  }
  

  }
  wb_robot_cleanup();

  return 0;
}
