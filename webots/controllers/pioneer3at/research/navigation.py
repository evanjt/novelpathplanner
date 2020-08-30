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
import research.constants as const
import research.clustering as clust

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


def prepare_to_map(robot, timestep, imu, wheels, targetBearing):

    # Loop for rotating the robot side on with the feature
    while robot.step(timestep) != -1:

        # Continually calculate and update robot bearing
        currentBearing = robot_bearing(imu)

        # Move the robot until side on with the target feature
        # Once side on mark the startingposition of the feature mapping
        if abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(wheels, -const.MAX_SPEED, const.MAX_SPEED)

        elif abs(targetBearing - currentBearing) > 1 \
                and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(wheels, const.MAX_SPEED, -const.MAX_SPEED)

        else:
            set_velocity(wheels, 0, 0)
            return

# NTS: need to improve mapping, use front sensor to avoid nothing loop
def feature_mapping(robot, timestep, wheels, gps, imu, lidar, hokuyo, width, threshold, i):

    # Identify which scan lines to use and calculate thresholds
    front = round(width/2)
    frontSide = round(width/3)
    side = round(width/6)
    threshold = threshold
    thresholdBuffer = threshold + 0.1
    frontSideThreshold = math.sqrt(threshold)
    frontSideThresholdBuffer = frontSideThreshold + 0.1
    frontThreshold = threshold
    frontThresholdBuffer = threshold + 0.1

    # Calculate starting position
    robot.step(timestep)
    currentPos = robot_position(gps)
    currentBearing = robot_bearing(imu)
    startingPos = currentPos
    lastScanPos = currentPos
    hasMoved = False
  
    # Capture first scan
    lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                        'lidar_feature' + str(i) + '.xyz')
    clust.capture_lidar_scene(robot, lidar, timestep, currentPos, currentBearing,
                path=lidar_feature_csvpath, scan='feature', threshold=threshold)

    # Loop for feature mapping
    while robot.step(timestep) != -1:

        # Continually calculate and update robot position
        # and distance to feature mapping start point
        # taking a new feature scan at a given threshold
        currentPos = robot_position(gps)
        currentBearing = robot_bearing(imu)
        distanceToStart = target_distance(currentPos, startingPos)
        values = hokuyo.getRangeImage()

        # Navigate the robot around the feature until it returns
        # to its starting point
        if target_distance(lastScanPos, currentPos) > \
            const.SCAN_THRESHOLD:
            clust.capture_lidar_scene(robot, lidar, timestep, currentPos, currentBearing,
                path=lidar_feature_csvpath, method='a', scan='feature',
                threshold=threshold)
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
                set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED*0.6)

            elif values[side] > thresholdBuffer \
                or values[frontSide] > frontSideThresholdBuffer \
                or values[front] > frontThreshold:
                set_velocity(wheels, const.MAX_SPEED*0.6, const.MAX_SPEED)

            else:
                set_velocity(wheels, const.MAX_SPEED, const.MAX_SPEED)


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
