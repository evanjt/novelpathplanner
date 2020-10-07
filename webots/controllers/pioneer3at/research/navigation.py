#!/usr/bin/python3

'''
    Code related to navigation methods for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import math
import time
import os
import logging
import csv
import numpy as np
import matplotlib.pyplot as plt

# Project specific functions
import research.constants as const
import research.clustering as clust
from research import pure_pursuit
import research.eta3_spline_path as eta3

def detect_obstacle(robot, hokuyo, width, halfWidth, rangeThreshold,
                    maxRange, braitenbergCoefficients):

    # Set obstacle counters
    leftObstacle = 0.0
    rightObstacle = 0.0

    # Scan using the Hokuyo
    values = hokuyo.getRangeImage()

    # Detect obstacle directly in front of the robot
    frontObstacle = values[math.floor(halfWidth)]

    # Detect obstacle scores based on Braitenberg coefficients
    for k in range(math.floor(halfWidth)):
        if values[k] < rangeThreshold:
            leftObstacle += braitenbergCoefficients[k] \
                            * (1.0 - values[k] / maxRange)

        j = width - k - 1

        if values[j] < rangeThreshold:
            rightObstacle += braitenbergCoefficients[k] \
                                * (1.0 - values[j] / maxRange)

    return leftObstacle, rightObstacle, leftObstacle + rightObstacle, frontObstacle

def lidar_mapping(pioneer3at, target, i, first_scan):

    # Define the mapping optimal mapping distance as a new variable
    threshold = target[2]

    # Identify which scan lines to use and calculate thresholds
    front = round(pioneer3at.hkfWidth/2)
    frontSide = round(pioneer3at.hkfWidth/3)
    side = round(pioneer3at.hkfWidth/6)
    thresholdBuffer = threshold + const.MAPPING_BUFFER
    frontSideThreshold = math.sqrt(pow(threshold,2)+pow(threshold,2))
    frontSideThresholdBuffer = frontSideThreshold + const.MAPPING_BUFFER
    frontThreshold = threshold
    frontThresholdBuffer = threshold + const.MAPPING_BUFFER

    # Calculate starting position
    pioneer3at.robot.step(pioneer3at.timestep)
    currentPos = robot_position(pioneer3at.gps)
    currentBearing = robot_bearing(pioneer3at.imu)
    startingPos = currentPos
    hasMoved = False
    counter = 0

    # Capture a new scan, rotate based on first scan, and write to file
    logging.debug("Acquiring LiDAR scan ...")
    scan = clust.capture_lidar_scene(pioneer3at, scan='feature', threshold=threshold)

    # 4D Transformation method using gps and imu for georeferencing scans
    T = np.eye(4)
    axisRotation = np.deg2rad((180 - currentBearing + 360) % 360)
    T[:3,:3] = scan.get_rotation_matrix_from_xyz((0,axisRotation, 0))
    T[0,3] = currentPos[0]
    T[1,3] = currentPos[1]
    T[2,3] = currentPos[2]
    transformed_scan = scan.transform(T)

    # Write the transformed scan to file
    lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                        'lidar_feature' + str(i+1) + 'scan' + str(counter+1) + '.xyz')
    clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)

    # Set the last_scan variable to the last transformed scan and set lastScanpos
    counter+=1
    lastScanPos = currentPos

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
        # to its starting point, scanning at the set threshold distance
        # and georeferencing those scans
        if target_distance(lastScanPos, currentPos) > \
            const.SCAN_THRESHOLD:

            logging.debug("Acquiring LiDAR scan ...")

            # Capture a new scan
            scan = clust.capture_lidar_scene(pioneer3at, scan='feature', threshold=threshold)

            # 4D Transformation method using gps and imu for georeferencing scans
            T = np.eye(4)
            axisRotation = np.deg2rad((180 - currentBearing + 360) % 360)
            T[:3,:3] = scan.get_rotation_matrix_from_xyz((0,axisRotation, 0))
            T[0,3] = currentPos[0]
            T[1,3] = currentPos[1]
            T[2,3] = currentPos[2]
            transformed_scan = scan.transform(T)

            # Write the transformed scan to file
            lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                                'lidar_feature' + str(i+1) + 'scan' + str(counter+1) + '.xyz')
            clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)

            # Set the last_scan variable to the last transformed scan and set lastScanpos
            lastScanPos = currentPos
            counter +=1

        elif hasMoved and distanceToStart < const.LOOP_THRESHOLD:
            logging.info("Mapping of feature {} complete".format(i+1))
            return
        elif not hasMoved and distanceToStart > const.LOOP_THRESHOLD:
            hasMoved = True
        else:
            if values[side] < threshold \
                or values[frontSide] < frontSideThreshold \
                or values[front] < frontThresholdBuffer:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED*0.6)
            elif (values[side] > thresholdBuffer and values[side] < thresholdBuffer + threshold*const.MAPPING_BUFFER) \
                or (values[frontSide] > frontSideThresholdBuffer and values[side] < thresholdBuffer + threshold*const.MAPPING_BUFFER)\
                or (values[front] > frontThreshold and values[side] < thresholdBuffer + threshold*const.MAPPING_BUFFER):
                set_velocity(pioneer3at.wheels, const.MAX_SPEED*0.6, const.MAX_SPEED)
            elif values[side] > thresholdBuffer \
                and values[frontSide] > frontSideThresholdBuffer \
                and values[front] > frontThresholdBuffer:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED*0.6, const.MAX_SPEED)
            else:
                set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)

def camera_mapping(pioneer3at, targets, featureNumber, first_scan):

    # Calculate starting position
    pioneer3at.robot.step(pioneer3at.timestep)
    currentPos = robot_position(pioneer3at.gps)
    currentBearing = robot_bearing(pioneer3at.imu)

    # Extract target information into variables
    bbox = targets[3]
    threshold = targets[2]
    scanBearing = targets[1]
    firstScanBearing = scanBearing

    # Variables for mapping control, as well as adding and indicating additional points along a plane
    mappingFlag = True
    bboxFlag=False
    edgeCounter = 0
    scanCounter = 0
    xlength = math.floor((bbox[1]-bbox[0])/const.SCAN_THRESHOLD)
    zlength = math.floor((bbox[3]-bbox[2])/const.SCAN_THRESHOLD)

    # Initialise and calculate the mapping positions
    mappingPositions = []
    [mappingPositions.append([]) for i in range(4)]

    if scanBearing == 270:
        [mappingPositions[0].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[1].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[2].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[3].append([bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
    elif scanBearing == 0:
        [mappingPositions[0].append([bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
        [mappingPositions[1].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[2].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[3].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
    elif scanBearing == 90:
        [mappingPositions[0].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[1].append([bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
        [mappingPositions[2].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[3].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
    elif scanBearing == 180:
        [mappingPositions[0].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[1].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[2].append([bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
        [mappingPositions[3].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
    else:
        logging.info("Bearing error encountered whilst mapping feature {}"
                    .format(featureNumber+1))

    # Save feature mapping locations to file
    with open(os.path.join(const.OUTPUT_PATH, str(featureNumber+1) +
              'mapping_points' + str(edgeCounter+1) + str(scanCounter+1) +
              '.xyz'), 'w', newline='') as outfile:
        csvwriter = csv.writer(outfile)
        for edge in mappingPositions:
            for point in edge:
                csvwriter.writerow(point)

    # Loop for feture mapping, where mapping positions are incrementally updated with each scan
    while mappingFlag is True:

        # Calculate starting position
        pioneer3at.robot.step(pioneer3at.timestep)
        currentPos = robot_position(pioneer3at.gps)
        currentBearing = robot_bearing(pioneer3at.imu)

        logging.debug("Acquiring Image and LiDAR scan ...")

        # Capture a new image and save to file
        cameraData = pioneer3at.camera.getImage()
        camera_feature_imagepath = os.path.join(const.OUTPUT_PATH,
                                            'camera_feature'+ str(featureNumber+1) + \
                                            'scan' + str(edgeCounter+1) + str(scanCounter+1) + \
                                            '.jpeg')
        pioneer3at.camera.saveImage(camera_feature_imagepath, 100)

        # Capture a new scan
        scan = clust.capture_lidar_scene(pioneer3at, scan='feature', threshold=threshold)
        # 4D Transformation method using gps and imu for georeferencing scans
        T = np.eye(4)
        axisRotation = np.deg2rad((180 - currentBearing + 360) % 360)
        T[:3,:3] = scan.get_rotation_matrix_from_xyz((0,axisRotation, 0))
        T[0,3] = currentPos[0]
        T[1,3] = currentPos[1]
        T[2,3] = currentPos[2]
        transformed_scan = scan.transform(T)

        # Increment scan counter
        scanCounter +=1

        # Output transformed scan to file
        lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
                                            'lidar_feature'+ str(featureNumber+1) + \
                                            'scan' + str(edgeCounter+1) + str(scanCounter+1) + \
                                            '.xyz')
        clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)

        logging.debug("Clustering LiDAR scan ...")

        # Detect features/clusters within the transformed lidar scan
        features = clust.cluster_points(np.array(transformed_scan.points))

        # Warn user if multiple features detected
        if len(features) > 1:
            logging.info("Warning one feature expected but multiple detected when mapping feature {} edge {}"
                        .format(featureNumber+1, edgeCounter+1))

        # Update the feature bbox dimensions based on the most recent scan
        for feature in features:
            xmax = max(feature, key=lambda x: x[0])[0]
            xmin = min(feature, key=lambda x: x[0])[0]
            ymax = max(feature, key=lambda x: x[1])[1]
            zmax = max(feature, key=lambda x: x[2])[2]
            zmin = min(feature, key=lambda x: x[2])[2]

            if xmin < bbox[0]:
                bbox[0] = xmin
            if xmax > bbox[1]:
                bbox[1] = xmax
            if zmin < bbox[2]:
                bbox[2] = zmin
            if zmax > bbox[3]:
                bbox[3] = zmax
            if ymax > threshold + 0.1:
                threshold = ymax
                logging.info("Height increase in feature {} detected, increasing mapping distance"
                            .format(featureNumber+1))

        # Determine if bbox changes indicate additional scans are required
        if math.floor((bbox[1]-bbox[0])/const.SCAN_THRESHOLD) > xlength:
            logging.info("Feature requires an additional scan(s) along the x axis")
            bboxFlag=True
            xlength = math.floor((bbox[1]-bbox[0])/const.SCAN_THRESHOLD)
        if math.floor((bbox[3]-bbox[2])/const.SCAN_THRESHOLD) > zlength:
            logging.info("Feature requires an additional scan(s) along the y axis")
            bboxFlag=True
            zlength = math.floor((bbox[3]-bbox[2])/const.SCAN_THRESHOLD)

        # If additional scans are required update mapping positions
        if bboxFlag:
            if firstScanBearing == 270:
                mappingPositions[0][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[1][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
                mappingPositions[2][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[3][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
            elif firstScanBearing == 0:
                mappingPositions[0][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
                mappingPositions[1][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[2][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
                mappingPositions[3][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
            elif firstScanBearing == 90:
                mappingPositions[0][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[1][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
                mappingPositions[2][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[3][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
            elif firstScanBearing == 180:
                mappingPositions[0][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
                mappingPositions[1][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[2][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
                mappingPositions[3][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
            else:
                logging.info("Bearing error encountered whilst mapping feature {}"
                            .format(featureNumber+1))

            # Save feature locations to file
            with open(os.path.join(const.OUTPUT_PATH, str(featureNumber+1) + \
                    'mapping_points' + str(edgeCounter+1) + str(scanCounter+1) + \
                    '.xyz'), 'w', newline='') as outfile:
                csvwriter = csv.writer(outfile)
                for edge in mappingPositions:
                    for point in edge:
                        csvwriter.writerow(point)

            # Reset the BBox flag
            bboxFlag = False

        # Check to see if the final scan has been completed for each edge
        # If complete either prepare for the next scan or set the mappingFlag to end the loop
        if ((scanBearing == 90 or scanBearing == 270) and scanCounter == zlength+1) or \
            ((scanBearing == 0 or scanBearing == 180) and scanCounter == xlength+1):
            logging.info("Mapping of edge {} feature {} complete"
                        .format(edgeCounter+1, featureNumber+1))
            scanCounter = 0
            edgeCounter +=1
            if edgeCounter == 4:
                logging.info("Mapping of feature {} complete"
                            .format(featureNumber+1))
                mappingFlag = False
            else:
                scanBearing = (scanBearing + 270) % 360
                nav_to_point_PID(pioneer3at,
                                x_goal = mappingPositions[edgeCounter][scanCounter][0],
                                y_goal = mappingPositions[edgeCounter][scanCounter][2],
                                theta_goal = np.radians(scanBearing), obstacle_detection = False)
        else:
                nav_to_point_PID(pioneer3at,
                                x_goal = mappingPositions[edgeCounter][scanCounter][0],
                                y_goal = mappingPositions[edgeCounter][scanCounter][2],
                                theta_goal = np.radians(scanBearing), obstacle_detection = False)

def zero_division(n, d):

    return n / d if d else 0

def get_velocity(wheels):

    left = abs((wheels[0].getVelocity() + wheels[2].getVelocity()))/2
    right = abs((wheels[1].getVelocity() + wheels[3].getVelocity()))/2

    return (left - right)

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
                     + (displacement[2])**2)

def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    beta is the angle between the robot's position and the goal position plus the goal angle

    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    Kp_beta*beta rotates the line so that it is parallel to the goal angle

    Move to specified pose

    Author: Daniel Ingram (daniel-s-ingram)
            Atsushi Sakai(@Atsushi_twi)

    P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

    """

    def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
        # Corners of triangular vehicle when pointing to the right (0 radians)
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        plt.plot(x_traj, y_traj, 'b--')

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        print(min(x_traj), max(x_traj))
        print(min(y_traj), max(y_traj))
        #print(y_traj)
        plt.xlim(min(x_traj)-1, max(x_traj)+1)
        plt.ylim(min(y_traj)-1, max(y_traj)+1)

        plt.pause(5)


    def transformation_matrix(x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])

    # Trajectory line fitting constants
    Kp_rho = const.MTP_KP_RHO
    Kp_alpha = const.MTP_KP_ALPHA
    Kp_beta = const.MTP_KP_BETA
    dt = const.MTP_DT

    plot_trajectory = const.MTP_PLOT

    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    rho = np.hypot(x_diff, y_diff)
    while rho > 0.001: #0.001
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y

        # Restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = Kp_rho * rho
        w = Kp_alpha * alpha + Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt

    if plot_trajectory:  # pragma: no cover
        plt.cla()
        plt.arrow(x_start, y_start, np.cos(theta_start),
                    np.sin(theta_start), color='r', width=0.1)
        plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                    np.sin(theta_goal), color='g', width=0.1)
        plot_vehicle(x, y, theta, x_traj, y_traj)

    traj_list = [(x, 0, y) for x, y in zip(x_traj, y_traj)]
    return traj_list

def nav_around_obstacle(pioneer3at):
    ''' This function is called if there is an obstacle. It takes over
        control of the robot and gets around the obstacle, when the threshold
        is sufficiently low as set by a threshold, it returns.
    '''

    while pioneer3at.robot.step(pioneer3at.timestep) != -1:
        obstacle_values = detect_obstacle(pioneer3at.robot, pioneer3at.hokuyoFront,
                                    pioneer3at.hkfWidth, pioneer3at.hkfHalfWidth,
                                    pioneer3at.hkfRangeThreshold, pioneer3at.hkfMaxRange,
                                    pioneer3at.hkfBraitenbergCoefficients)
        # If within obstacle, navigate around it
        if obstacle_values[2] > (const.OBSTACLE_THRESHOLD - const.OBSTACLE_BUFFER):

            speed_factor = (const.DECREASE_FACTOR * obstacle_values[2]) * (const.MAX_SPEED / obstacle_values[2])
            left_speed = speed_factor * obstacle_values[1]
            right_speed = speed_factor * obstacle_values[0]
            set_velocity(pioneer3at.wheels, right_speed, left_speed)

        elif obstacle_values[2] > const.OBSTACLE_END_LIMIT:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)
        # Sufficiently far away, return robot to full speed and return to program
        else:
            obstacle_flag = False
            logging.info("No obstacle")
            return obstacle_flag

def nav_to_point_PID(pioneer3at, x_goal, y_goal, theta_goal, obstacle_detection = True):
    ''' Uses Pure Pursuit path tracking, refer to main() in pure_pursuit.py for original
        code.
    '''
    x_start = robot_position(pioneer3at.gps)[0]
    y_start = robot_position(pioneer3at.gps)[2]
    theta_start = np.radians(robot_bearing(pioneer3at.imu))

    logging.info("Initial x: {:5.2f} m | y: {:5.2f} m | bearing: {:5.2f} deg".format(x_start, y_start, np.degrees(theta_start)))
    logging.info("Goal    x: {:5.2f} m | y: {:5.2f} m | bearing: {:5.2f} deg".format(x_goal, y_goal, np.degrees(theta_goal)))
    obstacle_flag = False
    traj_list = plan_eta_3_curve(x_start, y_start, theta_start + np.radians(const.TRAJPLANNING_BEARING_OFFSET),
                                 x_goal, y_goal, theta_goal + np.radians(const.TRAJPLANNING_BEARING_OFFSET))
    # traj_list = move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
    cx = [point[0] for point in traj_list]
    cy = [point[2] for point in traj_list]

    dt = 1/pioneer3at.timestep  # [s] time tick
    show_animation = const.PP_SHOW_ANIMATION
    show_velocity_animation = const.PP_SHOW_VELOCITY_ANIMATION

    #  target course
    target_speed = const.MAX_SPEED / 3.6  # [m/s]

    # initial state
    state = pure_pursuit.State(x=robot_position(pioneer3at.gps)[0],
                    y=robot_position(pioneer3at.gps)[2],
                    yaw=np.radians(robot_bearing(pioneer3at.imu) + const.TRAJPLANNING_BEARING_OFFSET),
                    v=get_velocity(pioneer3at.wheels))

    lastIndex = len(cx) - 1
    time = 0.0
    states = pure_pursuit.States()
    states.append(time, state)
    target_course = pure_pursuit.TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    while pioneer3at.robot.step(pioneer3at.timestep) != -1 and lastIndex > target_ind:
        obstacle_values = detect_obstacle(pioneer3at.robot, pioneer3at.hokuyoFront,
                                    pioneer3at.hkfWidth, pioneer3at.hkfHalfWidth,
                                    pioneer3at.hkfRangeThreshold, pioneer3at.hkfMaxRange,
                                    pioneer3at.hkfBraitenbergCoefficients)
        if obstacle_values[2] > const.OBSTACLE_THRESHOLD:
            obstacle_flag = True
        if obstacle_flag and obstacle_detection == True:
            logging.info("Found obstacle")
            return obstacle_flag, obstacle_values

        ai = pure_pursuit.proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit.pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update_pioneer(x=robot_position(pioneer3at.gps)[0],
                                y=robot_position(pioneer3at.gps)[2],
                                yaw=np.radians(robot_bearing(pioneer3at.imu) + const.TRAJPLANNING_BEARING_OFFSET),
                                v=get_velocity(pioneer3at.wheels), a=ai, delta=di)

        logging.debug("{:.2f} {:.2f}. x{:.2f} y{:.2f}".format(di, get_velocity(pioneer3at.wheels), robot_position(pioneer3at.gps)[0], robot_position(pioneer3at.gps)[2]))
        if di < 0:
            leftSpeed = const.MAX_SPEED - (abs(di) * const.PP_WEIGHT)
            rightSpeed = const.MAX_SPEED

        elif di > 0:
            leftSpeed = const.MAX_SPEED
            rightSpeed = const.MAX_SPEED - (abs(di) * const.PP_WEIGHT)

        else:
            leftSpeed = const.MAX_SPEED
            rightSpeed = const.MAX_SPEED

        set_velocity(pioneer3at.wheels, leftSpeed, rightSpeed)

        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            pure_pursuit.plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

        # Test
    assert lastIndex >= target_ind, "Cannot goal"

    if show_velocity_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

    return False, None

def plan_eta_3_curve(x_start=0, y_start=0, theta_start=0,
                 x_goal=8, y_goal=5, theta_goal=np.radians(180),
                 eta_shape = 0):
    ''' Builds a path for the pioneer3at.
        Theta is radians
    '''
    show_animation = False
    path_segments = []
    # segment 1: lane-change curve
    start_pose = [x_start, y_start, theta_start-const.ETA_OFFSET]
    end_pose = [x_goal, y_goal, theta_goal-const.ETA_OFFSET]
    # NOTE: The ordering on kappa is [kappa_A, kappad_A, kappa_B, kappad_B], with kappad_* being the curvature derivative
    kappa = [0, 0, 0, 0]
    eta = [0, 0, (eta_shape - 5) * 20, (5 - eta_shape) * 20, 0, 0]
    path_segments.append(eta3.eta3_path_segment(
        start_pose=start_pose, end_pose=end_pose, eta=eta, kappa=kappa))

    path = eta3.eta3_path(path_segments)

    # interpolate at several points along the path
    ui = np.linspace(0, len(path_segments), 1001)
    pos = np.empty((2, ui.size))
    for j, u in enumerate(ui):
        pos[:, j] = path.calc_path_point(u)
    #print(pos.shape)
    #print(list(pos[0]))
    if show_animation:
        # plot the path
        plt.plot(pos[0, :], pos[1, :])
        plt.pause(1.0)

    if show_animation:
        plt.close("all")

    x_traj = list(pos[0])
    y_traj = list(pos[1])

    traj_list = [(x, 0, y) for x, y in zip(x_traj, y_traj)]

    return traj_list
