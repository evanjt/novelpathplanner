#!/usr/bin/python3

"""pioneer3at controller."""

from controller import Robot, Lidar, GPS, InertialUnit, Camera, RangeFinder, DistanceSensor
import csv
import sys
import os
import math
import numpy as np


# Project specific functions
import research.clustering as clust
import research.constants as const
import research.navigation as nav

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
hkfMaxRange = hokuyoFront.getMaxRange() # 30
hkfRangeThreshold = hkfMaxRange / 20.0 #1.5
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
wheelNames = ['front left wheel', 'front right wheel','back left wheel', 'back right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# set the Braitenberg coefficient
hkfBraitenbergCoefficients = nav.getBraitenberg(robot, hkfWidth, hkfHalfWidth)

print("Pioneer is scanning surrounding area for features")

#wrap the below in a for loop based on user response to move and scan a new area

# Before beginning survey scan surrounds, cluster the scene, return the xy of clusters for feature mapping
roughFeatureList = []
featureList = []
point_array = clust.capture_lidar_scene(robot, lidar, timestep)

with open(os.path.join(const.OUTPUT_PATH,'features.csv'), 'w', newline='') as outfile:
    csvwriter = csv.writer(outfile)

    features = clust.cluster_points(point_array)

    for feature in features:

        center = len(feature)/2
        roughFeatureList.append(feature[round(center)])

        for pair in feature:

            pair.insert(1,0.1)
            csvwriter.writerow(pair)

    for roughPair in roughFeatureList:

        if len(featureList) == 0:
            featureList.append(roughPair)

        else:

            for i, finalPair in enumerate(featureList):

                if nav.xyDistance(roughPair, finalPair) < 0.1:
                    break

                elif i == len(featureList)-1:
                    featureList.append(roughPair)
                    csvwriter.writerow(roughPair)

    for ind, val in enumerate(featureList):
        const.TARGET_POSITIONS.insert(ind, val) # need to re-order based on dist at each step

with open(os.path.join(const.OUTPUT_PATH,'featurePoints.csv'), 'w', newline='') as outfile:
    csvwriter = csv.writer(outfile)

    for i in const.TARGET_POSITIONS:
        csvwriter.writerow(i)

print("%.d features found \nBeggining survey" %(len(const.TARGET_POSITIONS)-1))

# Loop through the target features provided
for i in range(len(const.TARGET_POSITIONS)):

    # Calculate initial bearing to target feature
    robot.step(timestep)
    currentPos = nav.robot_position(gps)
    targetBearing = nav.target_bearing(currentPos, const.TARGET_POSITIONS[i]) # need to reset bearing to feature every few meters to account for error in initial course
    flag = False

    # Navigate robot to the feature
    while robot.step(timestep) != -1:

        # Continually calculate and update robot position, bearing and distance to target feature
        currentPos = nav.robot_position(gps)
        currentBearing = nav.robot_bearing(imu)
        targetDistance = nav.target_distance(currentPos, const.TARGET_POSITIONS[i])

        # Continually detect obstacles
        obstacle = nav.detect_obstacle(robot, hokuyoFront, hkfWidth, hkfHalfWidth, hkfRangeThreshold,
                                       hkfMaxRange,  hkfBraitenbergCoefficients)

        # Once within range map the feature stop once returned home
        if targetDistance > 2 and obstacle[2] > const.OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - const.DECREASE_FACTOR * obstacle[2]) * const.MAX_SPEED / obstacle[2]
            nav.set_velocity(wheels, speed_factor * obstacle[0], speed_factor * obstacle[1])

        elif targetDistance > 2 and obstacle[2] > const.OBSTACLE_THRESHOLD-0.05:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)
            flag = False

        elif flag == False:
            targetBearing = nav.target_bearing(currentPos, const.TARGET_POSITIONS[i])
            flag = True

        elif targetDistance > 2 and abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 > 180:
            nav.set_velocity(wheels, const.MAX_SPEED*0.5, const.MAX_SPEED)

        elif targetDistance > 2 and abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 < 180:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)

        elif targetDistance > 2:
            nav.set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)

        elif i == len(const.TARGET_POSITIONS)-1:
            print("Survey complete")
            nav.set_velocity(wheels, 0, 0)
            break

        else:
            nav.prepare_to_map(robot, timestep, imu, wheels, (currentBearing + 90) % 360)
            nav.feature_mapping(robot, timestep, wheels, gps, hokuyoFront, hkfWidth, 2)
            break
