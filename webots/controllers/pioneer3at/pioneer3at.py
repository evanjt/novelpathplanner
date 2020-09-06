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

lidar = robot.getLidar('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

gps = robot.getGPS('gps')
gps.enable(timestep)

imu = robot.getInertialUnit('imu')
imu.enable(timestep)

cameraRange = robot.getRangeFinder("MultiSenseS21 meta range finder")
cameraRange.enable(timestep)
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
consoleformatter = logging.Formatter('%(asctime)s %(name)-5s: %(levelname)-5s %(message)s', datefmt='%H:%M:%S')
console.setFormatter(consoleformatter)
logging.getLogger('').addHandler(console)

# Start logging
thread = threading.Thread(target=log.worker, args=(gps,imu,)) # Send in GPS/IMU objects
thread.start()

# NTS: wrap below in a for loop based on need to move & map more areas
# remove HOME_LOCATION const and replace with user-entered variable
# need instructions for when no targets are found

logging.info("Pioneer is scanning surrounding area for features")
targets = clust.get_targets(robot, timestep, lidar, const.HOME_LOCATION)
log.write_featurepoints(targets, gps, imu)
logging.info("{} features found -- Beginning survey".format(len(targets)-1))

# Loop through the target features provided
for i in range(len(targets)):
    
    # Calculate initial bearing to target feature
    robot.step(timestep)
    startingPos = nav.robot_position(gps)
    targetBearing = nav.target_bearing(startingPos, targets[i][0])
    logging.info("Heading to feature {:1d} at: {:.3f}x, {:.3f}y, {:.3f}z".format(i+1, *targets[i][0]))
    logging.info("Along bearing: {:1f}".format(targetBearing))
    flag = False

    # Navigate robot to the feature
    while robot.step(timestep) != -1:
        
        # Continually calculate and update robot position,
        # bearing and distance to target feature
        currentPos = nav.robot_position(gps)
        currentBearing = nav.robot_bearing(imu)
        targetDistance = nav.target_distance(currentPos, targets[i][0])

        # Continually detect obstacles
        obstacle = nav.detect_obstacle(robot, hokuyoFront,
                                       hkfWidth, hkfHalfWidth,
                                       hkfRangeThreshold, hkfMaxRange,
                                       hkfBraitenbergCoefficients)

        # Once within range map the feature, stop once returned home
        if nav.target_distance(startingPos, currentPos) > \
            const.MOVEMENT_THRESHOLD:
            targetBearing = nav.target_bearing(currentPos, targets[i][0])
            startingPos = currentPos

        elif targetDistance > const.MAPPING_THRESHOLD \
                and obstacle[2] > const.OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - const.DECREASE_FACTOR * obstacle[2]) \
                            * const.MAX_SPEED / obstacle[2]
            nav.set_velocity(wheels, speed_factor * obstacle[0],
                             speed_factor * obstacle[1])

        elif targetDistance > const.MAPPING_THRESHOLD \
                and obstacle[2] > \
                    const.OBSTACLE_THRESHOLD - const.OBSTACLE_BUFFER:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)
            flag = False

        elif flag == False:
            targetBearing = nav.target_bearing(currentPos, targets[i][0])
            flag = True

        elif targetDistance > const.MAPPING_THRESHOLD \
                and abs(targetBearing - currentBearing) > const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 > 180:
            nav.set_velocity(wheels, const.MAX_SPEED*0.5, const.MAX_SPEED)

        elif targetDistance > const.MAPPING_THRESHOLD \
                and abs(targetBearing - currentBearing) > const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 < 180:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)

        elif targetDistance > const.MAPPING_THRESHOLD:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)

        elif i == len(targets)-1:
            print("Survey complete")
            nav.set_velocity(wheels, 0, 0)
            break

        else:
            logging.info("Starting to map feature #{} at: {:.3f}x, {:.3f}y, {:.3f}z".format(i+1, *currentPos))
            nav.prepare_to_map(robot, timestep, imu, wheels, targets[i][1])
            # NTS: flag when areas of the feature have not been mapped
            print(targets[i][3], targets[i][2])
            if const.DEVICE == 'lidar':
                nav.lidar_mapping(robot, timestep, wheels, gps, imu, lidar,
                                hokuyoFront, hkfWidth, targets[i][2], i)
            elif const.DEVICE == 'camera':
                nav.camera_mapping(robot, timestep, wheels, gps, imu, cameraRange,\
                    lidar, targets[i], i)
            print(targets[i][3], targets[i][2])
            break

# Join log together
thread.join()
