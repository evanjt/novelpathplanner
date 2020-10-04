#!/usr/bin/python3

'''
    Code related to the main controller of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

from controller import Robot
import math
import logging
import os
import numpy as np
import open3d as o3d

# Project specific functions
import research.clustering as clust
import research.constants as const
import research.navigation as nav
import research.logger as log
from research.classes import RobotDevice

# Define robot and devices
pioneer3at = RobotDevice(Robot())
pioneer3at.addLidar('lidar', 'HokuyoFront')
pioneer3at.addCamera('camera')
pioneer3at.addGPS('gps')
pioneer3at.addIMU('imu')
pioneer3at.addWheels(['front left wheel', 'front right wheel',
                      'back left wheel', 'back right wheel'])
pioneer3at.startLogging()

# Take the first simulation step
pioneer3at.robot.step(pioneer3at.timestep)

# Scan surrounding area and detect targets for mapping
logging.info("Pioneer is scanning surrounding area for features")
first_scan = clust.capture_lidar_scene(pioneer3at.robot,
                            pioneer3at.timestep,
                            pioneer3at.lidar,
                            const.HOME_LOCATION, 0)
clust.write_lidar_scene(first_scan)
clusters, targets = clust.get_targets(pioneer3at.robot,
                            pioneer3at.timestep,
                            pioneer3at.lidar,
                            pioneer3at.focalLength,
                            const.HOME_LOCATION,
                            np.array(first_scan.points))
for ind, val in enumerate(clusters):
    clust.write_lidar_scene(clust.convert_to_o3d(val), method='w',
                            path=os.path.join(const.OUTPUT_PATH,
                                              'cluster'+str(ind)+'.xyz'))
log.write_featurepoints(targets, pioneer3at.gps, pioneer3at.imu)
logging.info("{} features found -- Beginning survey".format(len(targets)-1))

# Loop through the detected target features
while pioneer3at.robot.step(pioneer3at.timestep) != -1:
    for i, target in enumerate(targets):

        # Calculate initial bearing to target feature
        logging.info("Heading to feature {} at: "
                     "{:.3f}x {:.3f}y {:.3f}z".format(i, *target[0]))
        logging.info("Along bearing: {:1f}".format(nav.target_bearing(nav.robot_position(pioneer3at.gps),
                                                                      target[0])))

        obstacle_flag = True # Test obstacles before start
        while obstacle_flag:
            obstacle_flag = nav.nav_around_obstacle(pioneer3at)
            obstacle_flag, obstacle_values = nav.nav_to_point_PID(pioneer3at,
                                                                  x_goal=target[0][0],
                                                                  y_goal=target[0][2],
                                                                  theta_goal=np.radians(target[1]))

        logging.info("Starting to map feature {} at: "
                    "{:.3f}x {:.3f}y {:.3f}z".format(i, *nav.robot_position(pioneer3at.gps)))

        if const.DEVICE == 'lidar':
            nav.lidar_mapping(pioneer3at, target, i, clust.convert_to_o3d(clusters[i]))
        elif const.DEVICE == 'camera':
            nav.camera_mapping(pioneer3at, target, i, clust.convert_to_o3d(clusters[i]))

    # All features mapped, shutdown
    logging.info("Survey complete")
    nav.set_velocity(pioneer3at.wheels, 0, 0)
    pioneer3at.endLogging()
    break
