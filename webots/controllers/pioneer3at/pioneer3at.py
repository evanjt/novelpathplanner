#!/usr/bin/python3

'''
    Code related to the main controller of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

from controller import Robot
import logging
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
pioneer3at.addGPS('gps')
pioneer3at.addIMU('imu')
pioneer3at.addWheels(['front left wheel', 'front right wheel',
                      'back left wheel', 'back right wheel'])
pioneer3at.startLogging()

# Take the first simulation step
pioneer3at.robot.step(pioneer3at.timestep)

# Scan surrounding area and detect targets for mapping
# NTS: instead of using first_scan in the rotation using only the relevant cluster
logging.info("Pioneer is scanning surrounding area for features")
first_scan = clust.capture_lidar_scene(pioneer3at.robot,
                            pioneer3at.timestep,
                            pioneer3at.lidar,
                            const.HOME_LOCATION, 0)
clust.write_lidar_scene(first_scan)
clusters, targets = clust.get_targets(pioneer3at.robot,
                            pioneer3at.timestep,
                            pioneer3at.lidar,
                            const.HOME_LOCATION,
                            np.array(first_scan.points))
log.write_featurepoints(targets, pioneer3at.gps, pioneer3at.imu)
logging.info("{} features found -- Beginning survey".format(len(targets)-1))

# Loop through the detected target features
while pioneer3at.robot.step(pioneer3at.timestep) != -1:
    for i, target in enumerate(targets):
        
        # Calculate initial bearing to target feature
        pioneer3at.robot.step(pioneer3at.timestep)
        startingPos = nav.robot_position(pioneer3at.gps)
        targetBearing = nav.target_bearing(startingPos, target[0])
        logging.info("Heading to feature {} at: "
                     "{:.3f}x {:.3f}y {:.3f}z".format(i, *target[0]))
        logging.info("Along bearing: {:1f}".format(targetBearing))
        flag = False

        # Start navigation to a point
        currentPos, currentBearing = nav.nav_to_point(i, target,
                                                      pioneer3at, flag,
                                                      startingPos,
                                                      targetBearing)
        if i==len(targets)-1:
            break
        else:
            logging.info("Starting to map feature {} at: "
                        "{:.3f}x {:.3f}y {:.3f}z".format(i, *currentPos))
            nav.prepare_to_map(pioneer3at, target[1])
            
            # Convert points to open 3D point cloud    
            xyz = np.array(clusters[i])
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            
            # NTS: flag when areas of the feature have not been mapped
            if const.DEVICE == 'lidar':
                nav.lidar_mapping(pioneer3at, targets[i][2], i, pcd)
            elif const.DEVICE == 'camera':
                nav.camera_mapping(pioneer3at, targets[i], i, pcd)

    # After list is done, shutdown
    logging.info("Survey complete")
    nav.set_velocity(pioneer3at.wheels, 0, 0)
    break
