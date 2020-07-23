/* Controller for ENGR90037 Husky navigation
   Authors: Evan Thomas evant1@student.unimelb.edu.au
            Josh Clough cloughj@student.unimelb.edu.au
     
   To be executed as a controller within a Webots simulator
*/

 
 
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pwd.h>
#include <unistd.h>
#include <string.h>

#include <webots/motor.h>
#include <webots/robot.h>
// #include <webots/supervisor.h>
#include <webots/lidar.h>
#include <webots/gps.h>

#define TIME_STEP 64
#define ROBOT_HEIGHT 1.0 // As to not include points beneath
#define VALUES_IN_GNSS 3

// Calculates the distance between the LiDAR and a point with
// euclidean distance. LiDAR relative to a point is always (0, 0, 0)
double euclidean_dist_from_LiDAR(double x, double y, double z) {
  return sqrt((x*x) + (y*y) + (z*z));
}

int main(int argc, char **argv) {
  int time_step = 0;
  WbDeviceTag velodyne;
  WbDeviceTag gnss;
  struct passwd *pw = getpwuid(getuid());  // Get home dir of user
  const char *userhomedir = pw->pw_dir;    // And assign it here
  // int velodyne_sampling_period = 0;

  // Initialise robot
  wb_robot_init();
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

  // init motors
  wb_motor_set_position(front_left_wheel, INFINITY);
  wb_motor_set_position(front_right_wheel, INFINITY);
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);

  // init speed for each wheel
  double back_left_speed = 0.0, back_right_speed = 0.0;
  double front_left_speed = 0.0, front_right_speed = 0.0;
  wb_motor_set_velocity(front_left_wheel, front_left_speed);
  wb_motor_set_velocity(front_right_wheel, front_right_speed);
  wb_motor_set_velocity(back_left_wheel, back_left_speed);
  wb_motor_set_velocity(back_right_wheel, back_right_speed);
  
  // init dynamic variables
  // double left_obstacle = 0.0, right_obstacle = 0.0;

  // Setup the Velodyne LiDAR
  time_step = wb_robot_get_basic_time_step();
  velodyne = wb_robot_get_device("Velodyne HDL-32E");
  gnss = wb_robot_get_device("SwiftNav RTK");
  
  // Enable sensors
  wb_lidar_enable(velodyne, time_step);
  wb_gps_enable(gnss, time_step);
  // velodyne_sampling_period = wb_lidar_get_sampling_period(velodyne);

  // Velodyne status messages
  // if (velodyne_sampling_period == 0) {
    // printf("Velodyne disabled\n");
  // }
  // else {
    // printf("Velodyne sampling period: %d\n", velodyne_sampling_period);
  // }
  
  // Test the calculation function here
  //printf("Dist (3,3,3) to (0,0,0): %f\n", euclidean_dist_from_LiDAR(3,3,3)); //Test
 
  wb_lidar_enable_point_cloud(velodyne);
  // int is_lidar = 0;
  // is_lidar = wb_lidar_is_point_cloud_enabled(velodyne);
  // printf("%d\n", is_lidar);
  // printf("Point Count %d\n", point_count);
  
  // Put file in user home dir (linux not sure for windows)
  FILE *fp;
  char *filename = "/points.csv";
  char filepath[100];
  strcpy(filepath, userhomedir);
  strcat(filepath, filename);
  fp = fopen(filepath, "w");
  
  // Run through the time steps in the robot. Stop if Webots says -1
  while (wb_robot_step(TIME_STEP) != -1) {
  
    // Assign point stream from Velodyne
    const int point_count = wb_lidar_get_number_of_points(velodyne);
    const WbLidarPoint* point_stream = wb_lidar_get_point_cloud(velodyne); 
    const double* gnss_stream = wb_gps_get_values(gnss);
    double gnss_location[VALUES_IN_GNSS];
    
    // Move robot forwards
    wb_motor_set_velocity(front_left_wheel, 5);
    wb_motor_set_velocity(front_right_wheel, 5);
    wb_motor_set_velocity(back_left_wheel, 5);
    wb_motor_set_velocity(back_right_wheel, 5);
    
    // Three values in stream for x, y, z. Unpack in the loop    
    for (int i = 0; i < VALUES_IN_GNSS; i++) {
      // printf("%f\n", gnss_stream[i]);
      gnss_location[i] = *gnss_stream;
      gnss_stream++;
    }
    printf("GNSS: %f, %f, %f\n", gnss_location[0], 
                           gnss_location[1], 
                           gnss_location[2]);    
                           
    for (int i = 0; i < point_count; i++) {
      if (point_stream->y > -ROBOT_HEIGHT 
         && point_stream->x != 0.0 
         && point_stream->y != 0.0
         && point_stream->z != 0.0) {
         // double x = point_stream->x + gnss_location[0];
         // double y = point_stream->y + gnss_location[1];
         // double z = point_stream->z + gnss_location[2];         
         double x = point_stream->x;
         double y = point_stream->y;
         double z = point_stream->z;
//        fprintf(fp, "%.2f\t%.2f\t%.2f\t%.2f\t%d\t%.2f\n", 
//                x, y, z, point_stream->time, point_stream->layer_id, 
//                euclidean_dist_from_LiDAR(x, y, z));
      }
      point_stream++;  // Increment the stream
    }
   };
   
   fclose(fp);
   wb_robot_cleanup();
  return 0;
}
