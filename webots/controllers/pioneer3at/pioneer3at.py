#!/usr/bin/python3

'''
    Code related to the main controller of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

from controller import Robot
import csv
import os
import logging
import threading
import time

# Project specific functions
import research.clustering as clust
import research.constants as const
import research.navigation as nav
import research.logger as log

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get and enable robot devices
hokuyoFront = robot.getLidar("HokuyoFront")
hokuyoFront.enable(timestep)
hokuyoFront.enablePointCloud()
hkfWidth = hokuyoFront.getHorizontalResolution()
hkfHalfWidth = hkfWidth / 2.0
hkfMaxRange = hokuyoFront.getMaxRange()  # 30
hkfRangeThreshold = hkfMaxRange / 20.0  # 1.5
hkfValues = []

# hokuyoRear = robot.getLidar("HokuyoRear")
# hokuyoRear.enable(timestep)
# hokuyoRear.enablePointCloud()

lidar = robot.getLidar('Velodyne HDL-32E')
lidar.enable(timestep)
lidar.enablePointCloud()
lidar.setFrequency(20)

gps = robot.getGPS('gps')
gps.enable(timestep)

imu = robot.getInertialUnit('imu')
imu.enable(timestep)

# cameraRange = robot.getRangeFinder("MultiSenseS21 meta range finder")
# cameraRange.enable(timestep)
# cameraCenter = robot.getCamera("MultiSenseS21 meta camera")
# cameraCenter.enable(timestep)
# cameraLeft = robot.getCamera("MultiSenseS21 left camera")
# cameraLeft.enable(timestep)
# cameraRight = robot.getCamera("MultiSenseS21 right camera")
# cameraRight.enable(timestep)

wheels = []
wheelNames = ['front left wheel', 'front right wheel',
              'back left wheel', 'back right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# set the Braitenberg coefficient
hkfBraitenbergCoefficients = nav.getBraitenberg(robot, hkfWidth, hkfHalfWidth)

# Set up logger with formatting for file output
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-5s %(levelname)-5s %(message)s',
                    datefmt='%m-%d %H:%M:%S',
                    filename=os.path.join(const.OUTPUT_PATH, const.LOGFILENAME),
                    filemode='w')
console = logging.StreamHandler()
console.setLevel(logging.INFO)
# Formatting for console output
consoleformatter = logging.Formatter('%(asctime)s %(name)-5s: %(levelname)-5s %(message)s', datefmt='%H:%M:%S')
console.setFormatter(consoleformatter)
logging.getLogger('').addHandler(console)

#info = {'stop': False}
thread = threading.Thread(target=log.worker, args=(gps,imu,)) # Send in GPS/IMU objects
thread.start()

# NTS: wrap below in a for loop based on need to move & map more areas
# remove HOME_LOCATION const and replace with user-entered variable

logging.info("Pioneer is scanning surrounding area for features")

targets = clust.get_targets(robot, timestep, lidar)
#print(targets)
log.write_featurepoints(targets, gps, imu)

# NTS: Need to calculate based on feature height
# need to do dbscan in 3d for this and
# need to return feature points and feature heights
mappingDistance = 2
logging.info("{} features found -- Beginning survey".format(len(targets)-1))

# Loop through the target features provided
for i in range(len(targets)):
    # Calculate initial bearing to target feature
    robot.step(timestep)
    currentPos = nav.robot_position(gps)
     # NTS: need to reset bearing to feature every few meters
     # to account for error in initial course
    targetBearing = nav.target_bearing(currentPos, targets[i])
    print(targetBearing)
    flag = False


    # Navigate robot to the feature
    while robot.step(timestep) != -1:
        # Continually calculate and update robot position,
        # bearing and distance to target feature
        currentPos = nav.robot_position(gps)
        currentBearing = nav.robot_bearing(imu)
        targetDistance = nav.target_distance(currentPos, targets[i])

        # Continually detect obstacles
        obstacle = nav.detect_obstacle(robot, hokuyoFront,
                                       hkfWidth, hkfHalfWidth,
                                       hkfRangeThreshold, hkfMaxRange,
                                       hkfBraitenbergCoefficients)

        # Once within range map the feature, stop once returned home
        if targetDistance > mappingDistance \
                and obstacle[2] > const.OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - const.DECREASE_FACTOR * obstacle[2]) \
                            * const.MAX_SPEED / obstacle[2]
            nav.set_velocity(wheels, speed_factor * obstacle[0],
                             speed_factor * obstacle[1])

        elif targetDistance > mappingDistance \
                and obstacle[2] > const.OBSTACLE_THRESHOLD-0.05:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)
            flag = False

        elif flag == False:
            targetBearing = nav.target_bearing(currentPos, targets[i])
            flag = True

        elif targetDistance > mappingDistance \
                and abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 > 180:
            nav.set_velocity(wheels, const.MAX_SPEED*0.5, const.MAX_SPEED)

        elif targetDistance > mappingDistance \
                and abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 < 180:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)

        elif targetDistance > mappingDistance:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)

        elif i == len(targets)-1:
            print("Survey complete")
            nav.set_velocity(wheels, 0, 0)
            break

        else:
            # NTS: change function to be on right angle with feature,
            # need to obtain the bearing of the feature plane to do this
            logging.info("Start mapping feature")
            nav.prepare_to_map(robot, timestep, imu, wheels,
                               (currentBearing + 90) % 360)
            nav.feature_mapping(robot, timestep, wheels, gps,
                                hokuyoFront, hkfWidth, mappingDistance)
            break

# Join log together
thread.join()
