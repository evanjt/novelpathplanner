#!/usr/bin/python3

'''
    Code related to navigation methods for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import math
import os
import logging
import numpy as np
import research.constants as const
import research.clustering as clust
import research.slam as slam

def nav_to_point(i, target, pioneer3at, flag, startingPos, targetBearing, first_scan):

    # Set the last_scan variable to first lidar scan
    last_scan = first_scan
    counter = 0

    # Navigate robot to the feature
    while pioneer3at.robot.step(pioneer3at.timestep) != -1:

        # Continually calculate and update robot position,
        # bearing and distance to target feature
        currentPos = robot_position(pioneer3at.gps)
        currentBearing = robot_bearing(pioneer3at.imu)
        targetDistance = target_distance(currentPos, target[0])

        # Continually detect obstacles
        obstacle = detect_obstacle(pioneer3at.robot, pioneer3at.hokuyoFront,
                                   pioneer3at.hkfWidth, pioneer3at.hkfHalfWidth,
                                   pioneer3at.hkfRangeThreshold, pioneer3at.hkfMaxRange,
                                   pioneer3at.hkfBraitenbergCoefficients)

        # Once within range map the feature, stop once returned home
        if target_distance(startingPos, currentPos) > \
            const.MOVEMENT_THRESHOLD:
            targetBearing = target_bearing(currentPos, target[0])
            startingPos = currentPos

            # Intermittently capture a new scan based on the movement threshold
            # Rotate that scan based on the previous scan, and write to file
            # logging.debug("Acquiring LiDAR scan ...")
            # scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
            #                                         pioneer3at.lidar, currentPos,
            #                                         currentBearing)
            # transformed_scan = slam.rotate_scan(last_scan, scan)
            # lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
            #                                     'scan'+str(counter)+'.xyz')
            # clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)
            # last_scan = transformed_scan
            # counter+=1

        elif targetDistance > const.MAPPING_DISTANCE and obstacle[2] > \
            const.OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - (const.DECREASE_FACTOR * obstacle[2])) * \
                                            (const.MAX_SPEED / obstacle[2])
            set_velocity(pioneer3at.wheels, speed_factor * obstacle[0],
                             speed_factor * obstacle[1])

        elif targetDistance > const.MAPPING_DISTANCE \
                and obstacle[2] > const.OBSTACLE_THRESHOLD-0.05:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)
            flag = False

        elif flag == False:
            targetBearing = target_bearing(currentPos, target[0])
            flag = True

        elif targetDistance > const.MAPPING_DISTANCE \
                and abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED*0.5, const.MAX_SPEED)

        elif targetDistance > const.MAPPING_DISTANCE \
                and abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)

        elif targetDistance > const.MAPPING_DISTANCE:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)

        else:
            return currentPos, currentBearing

def detect_obstacle(robot, hokuyo, width, halfWidth, rangeThreshold,
                    maxRange, braitenbergCoefficients):

    # set obstacle counters
    leftObstacle = 0.0
    rightObstacle = 0.0

    # scan using the Hokuyo
    values = hokuyo.getRangeImage()

    # detect obstacle scores based on Braitenberg coefficients
    for k in range(math.floor(halfWidth)):
        if values[k] < rangeThreshold:
            leftObstacle += braitenbergCoefficients[k] \
                            * (1.0 - values[k] / maxRange)

        j = width - k - 1

        if values[j] < rangeThreshold:
            rightObstacle += braitenbergCoefficients[k] \
                                * (1.0 - values[j] / maxRange)

    return leftObstacle, rightObstacle, leftObstacle + rightObstacle

def prepare_to_map(pioneer3at, targetBearing):

    # Loop for rotating the robot side on with the feature
    while pioneer3at.robot.step(pioneer3at.timestep) != -1:

        # Continually calculate and update robot bearing
        currentBearing = robot_bearing(pioneer3at.imu)

        # Move the robot until side on with the target feature
        # Once side on mark the startingposition of the feature mapping
        if abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(pioneer3at.wheels, -const.MAX_SPEED, const.MAX_SPEED)

        elif abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, -const.MAX_SPEED)

        else:
            set_velocity(pioneer3at.wheels, 0, 0)
            return

# NTS: need to improve mapping, use front sensor to avoid nothing loop
def lidar_mapping(pioneer3at, threshold, i, first_scan):

    # Identify which scan lines to use and calculate thresholds
    front = round(pioneer3at.hkfWidth/2)
    frontSide = round(pioneer3at.hkfWidth/3)
    side = round(pioneer3at.hkfWidth/6)
    thresholdBuffer = threshold + 0.1
    frontSideThreshold = math.sqrt(threshold)
    frontSideThresholdBuffer = frontSideThreshold + 0.1
    frontThreshold = threshold
    frontThresholdBuffer = threshold + 0.1

    # Calculate starting position
    pioneer3at.robot.step(pioneer3at.timestep)
    currentPos = robot_position(pioneer3at.gps)
    currentBearing = robot_bearing(pioneer3at.imu)
    startingPos = currentPos
    lastScanPos = currentPos
    hasMoved = False

    # Set the last_scan variable to first lidar scan
    last_scan = first_scan

    # Capture a new scan, rotate based on first scan, and write to file
    scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                    pioneer3at.lidar, currentPos, currentBearing,
                                    scan='feature', threshold=threshold)
    transformed_scan = slam.rotate_scan(last_scan, scan)
    lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                        'lidar_feature' + str(i) + '.xyz')
    clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)

    # Loop for feature mapping
    while pioneer3at.robot.step(pioneer3at.timestep) != -1:

        # Continually calculate and update robot position
        # and distance to feature mapping start point
        # taking a new feature scan at a given threshold
        currentPos = robot_position(pioneer3at.gps)
        currentBearing = robot_bearing(pioneer3at.imu)
        distanceToStart = target_distance(currentPos, startingPos)
        values = pioneer3at.hokuyoFront.getRangeImage()

        # Navigate the robot around the feature until it returns
        # to its starting point
        if target_distance(lastScanPos, currentPos) > \
            const.SCAN_THRESHOLD:
            last_scan = transformed_scan # ??
            scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                             pioneer3at.lidar, currentPos, currentBearing,
                                             method='a', scan='feature', threshold=threshold) # Should this variable be called something else?
            slam.rotate_scan(last_scan, transformed_scan) # Rotating it to itself? 4 lines up this becomes the same variable, should this become transformed_scan?
            clust.write_lidar_scene(transformed_scan, method='a', # Writing transformed_scan, but last_scan is transformed_scan
                                    path=lidar_feature_csvpath)
            lastScanPos = currentPos

        elif hasMoved and distanceToStart < const.LOOP_THRESHOLD:
            logging.info("Mapped feature #{} and stored points in CSV".format(i+1))
            return

        elif not hasMoved and distanceToStart > const.LOOP_THRESHOLD:
            hasMoved = True

        else:
            if values[side] < threshold \
                or values[frontSide] < frontSideThreshold \
                or values[front] < frontThreshold:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED*0.6)

            elif values[side] > thresholdBuffer \
                or values[frontSide] > frontSideThresholdBuffer \
                or values[front] > frontThreshold:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED*0.6, const.MAX_SPEED)

            else:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)

def camera_mapping(pioneer3at, targets, i, first_scan):

    # extract target information into variables
    bbox = targets[3]
    threshold = targets[2]
    scanBearing = targets[1]

    # Calculate starting position
    pioneer3at.robot.step(pioneer3at.timestep)
    currentPos = robot_position(pioneer3at.gps)
    currentBearing = robot_bearing(pioneer3at.imu)

    # Set the last_scan variable to first lidar scan
    last_scan = first_scan

    # Capture a new scan, rotate based on first scan, and write to file
    logging.debug("Acquiring LiDAR scan ...")
    scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                            pioneer3at.lidar, currentPos,
                                            currentBearing, scan='feature', threshold=threshold)
    transformed_scan = slam.rotate_scan(last_scan, scan)
    lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                        'lidar_feature' + str(i) + '.xyz')
    clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)

    # Detect features/clusters within lidar scene
    logging.debug("Clustering LiDAR scan ...")
    features = clust.cluster_points(np.array(transformed_scan.points))

    if len(features) > 1:
        print("Warning one feature expected but multiple detected")

    # NTS: need to properly rotate the cloud for accurate bbox update
    # NTS: could potentially be other problems with the bbox update
    for feature in features:
        xmax = max(feature, key=lambda x: x[0])[0]
        xmin = min(feature, key=lambda x: x[0])[0]
        ymax = max(feature, key=lambda x: x[1])[1]
        zmax = max(feature, key=lambda x: x[2])[2]
        zmin = min(feature, key=lambda x: x[2])[2]

    if xmin < bbox[0]:
        bbox[0] = xmin
        print("xmin changed")
    if xmax > bbox[1]:
        bbox[1] = xmax
        print("xmax changed")
    if zmin < bbox[2]:
        bbox[2] = zmin
        print("zmin changed")
    if zmax > bbox[3]:
        bbox[3] = zmax
        print("zmax changed")
    if ymax > threshold:
        threshold = ymax
        print("mapping dist changed")

    # NTS: sections of the robot which are being scanned are affecting bbox changes

    # NTS: once bbox is updated need to calc if there needs to be a change to mapping locations based on bbox dimensions and mappingdist

    # NTS: need to first calculate rough mapping locations based on initial scan

    # NTS: once mapping locations have been updated smoothly move to the next point using ET1's trajectory function and take a photo, then re-check the bbox and mappingdist and repat until return to starting location

def getBraitenberg(robot, width, halfWidth):

    braitenbergCoefficients = []

    for i in range(math.floor(width)):
        braitenbergCoefficients.append(gaussian(i, halfWidth, width / 5))

    return braitenbergCoefficients

def gaussian(x, mu, sigma):

    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) \
            * math.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))

def set_velocity(wheels, leftSpeed, rightSpeed):

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

def robot_position(gps):

    currentGPSPos = gps.getValues()
    currentPos = location_offset(currentGPSPos, 0, -0.3, -0.2)

    return currentPos

def robot_bearing(imu):

    return (math.degrees(imu.getRollPitchYaw()[2]) * -1 + 360) % 360

def target_bearing(current, target):

    displacement = difference(current, target)
    bearingToTarget = bearing(displacement)

    return bearingToTarget

def target_distance(current, target):

    displacement = difference(current, target)
    distanceToTarget = distance(displacement)

    return distanceToTarget

def location_offset(position, x, y, z):

    return (position[0]+x, position[1]+y, position[2]+z)

# Should these be opposite as to the inputs that are calling it?
def difference(target, current):

    return (target[0] - current[0],
            target[1] - current[1],
            target[2] - current[2])

def bearing(displacement):

    initial_bearing = math.degrees(math.atan2(displacement[0],
                                              displacement[2]))
    compass_bearing = (initial_bearing * -1 + 180) % 360

    return compass_bearing

def distance(displacement):

    return math.sqrt((displacement[0])**2
                     + (displacement[1])**2
                     + (displacement[2])**2)

def elevation(displacement, dist):

    return math.asin(displacement[2] / dist)

def xyDistance(pair1, pair2):

    displacement = pair1[0] - pair2[0], pair1[1] - pair2[1]

    return math.sqrt((displacement[0])**2 + (displacement[1])**2)
