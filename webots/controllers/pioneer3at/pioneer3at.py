#!/usr/bin/python3

'''
    Code related to the main controller of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

from controller import Robot
import logging

# Project specific functions
import research.clustering as clust
import research.constants as const
import research.navigation as nav
import research.logger as log
from research.classes import RobotDevice

pioneer3at = RobotDevice(Robot())
pioneer3at.addLidar('lidar', 'HokuyoFront')
pioneer3at.addGPS('gps')
pioneer3at.addIMU('imu')
pioneer3at.addWheels(['front left wheel', 'front right wheel',
                      'back left wheel', 'back right wheel'])
pioneer3at.startLogging()

# Find targets to scan
logging.info("Pioneer is scanning surrounding area for features")
targets = clust.get_targets(pioneer3at.robot,
                            pioneer3at.timestep,
                            pioneer3at.lidar,
                            const.HOME_LOCATION)
log.write_featurepoints(targets, pioneer3at.gps, pioneer3at.imu)
logging.info("{} features found -- Beginning survey".format(len(targets)-1))

# Loop through the target features provided
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
        logging.info("Starting to map feature {} at: "
                     "{:.3f}x {:.3f}y {:.3f}z".format(i, *currentPos))
        nav.prepare_to_map(pioneer3at, target[1])

        # NTS: flag when areas of the feature have not been mapped
        nav.feature_mapping(pioneer3at, target[2], i)

    # After list is done, shutdown
    logging.info("Survey complete")
    nav.set_velocity(pioneer3at.wheels, 0, 0)
    break
