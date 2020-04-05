/* Controller for ENGR90037 Husky navigation
   Authors: Evan Thomas evant1@student.unimelb.edu.au
            Josh Clough cloughj@student.unimelb.edu.au
     
   To be executed as a controller within a Webots simulator
*/

 
 
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/lidar.h>

static int time_step = 0;
static WbDeviceTag velodyne;
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define ROBOT_HEIGHT 1.0 // As to not include points beneath

// Calculates the distance between the LiDAR and a point with
// euclidean distance. LiDAR relative to a point is always (0, 0, 0)
double euclidean_dist_from_LiDAR(double x, double y, double z) {
  return sqrt((x*x) + (y*y) + (z*z));
}

int main(int argc, char **argv) {
  // int velodyne_sampling_period = 0;
  
  // Initialise robot
  wb_robot_init();
  
  // Setup the Velodyne LiDAR
  time_step = wb_robot_get_basic_time_step();
  velodyne = wb_robot_get_device("Velodyne HDL-32E");
  wb_lidar_enable(velodyne, time_step);
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
  
  
  FILE * fp;
  fp = fopen("points.csv", "w");
  // Run through the time steps in the robot. Stop if Webots says -1
  while (wb_robot_step(TIME_STEP) != -1) {
  
    // Assign point stream from Velodyne
    const int point_count = wb_lidar_get_number_of_points(velodyne);
    const WbLidarPoint* point_stream = wb_lidar_get_point_cloud(velodyne); 
    for (int i = 0; i < point_count; i++) {
      if (point_stream->y > -1.0 
         && point_stream->x != 0.0 
         && point_stream->y != 0.0
         && point_stream->z != 0.0) {
        fprintf(fp, "%.2f\t%.2f\t%.2f\t%.2f\t%d\t%.2f\n", point_stream->time, 
                point_stream->x, point_stream->y, point_stream->z,
                point_stream->layer_id, 
                euclidean_dist_from_LiDAR(point_stream->x,
                point_stream->y, point_stream->z));
      }
      point_stream++;  // Increment the stream
    }
    
    
    
   };
   
   fclose(fp);
  return 0;
}
