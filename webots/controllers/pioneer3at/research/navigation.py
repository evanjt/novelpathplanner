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
import csv
import numpy as np

# Project specific functions
import research.constants as const
import research.clustering as clust
import research.slam as slam

def nav_to_point(i, target, pioneer3at, flag, startingPos, first_scan):

    # Set the last_scan variable to first lidar scan
    last_scan = first_scan
    counter = 0
    
    # calc bearing to target
    targetBearing = target_bearing(startingPos, target)

    # Navigate robot to the feature
    while pioneer3at.robot.step(pioneer3at.timestep) != -1:

        # Continually calculate and update robot position,
        # bearing and distance to target feature
        currentPos = robot_position(pioneer3at.gps)
        currentBearing = robot_bearing(pioneer3at.imu)
        targetDistance = target_distance(currentPos, target)

        # Continually detect obstacles
        obstacle = detect_obstacle(pioneer3at.robot, pioneer3at.hokuyoFront,
                                   pioneer3at.hkfWidth, pioneer3at.hkfHalfWidth,
                                   pioneer3at.hkfRangeThreshold, pioneer3at.hkfMaxRange,
                                   pioneer3at.hkfBraitenbergCoefficients)

        # Once within range map the feature, stop once returned home
        if target_distance(startingPos, currentPos) > \
            const.MOVEMENT_THRESHOLD:

            # Set new startinPos and reset targetBearing
            targetBearing = target_bearing(currentPos, target)
            startingPos = currentPos

            # The below code can be commented out to avoid additional scans at a set interval###
            # Capture a new scan, rotate based on first scan, and write to file
            # logging.debug("Acquiring LiDAR scan ...")
            # scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
            #                                         pioneer3at.lidar, currentPos,
            #                                         currentBearing)
            #                                         # , scan='feature', threshold=threshold)
            # transformed_scan = slam.rotate_scan(last_scan, scan)
            # lidar_feature_csvpath = os.path.join(const.OUTPUT_PATH,
            #                                     'nav2feature' + str(i) + 'scan' + str(counter) + '.xyz')
            # clust.write_lidar_scene(transformed_scan, path=lidar_feature_csvpath)
            
            # # Set the last_scan variable to the last transformed scan
            # last_scan = transformed_scan
            # counter +=1

            #####################################################################################

        elif targetDistance > const.MAPPING_DISTANCE and obstacle[3] <= 1.5:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, -const.MAX_SPEED)
        
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
            targetBearing = target_bearing(currentPos, target)
            flag = True

        elif targetDistance > const.MAPPING_DISTANCE \
                and abs(targetBearing - currentBearing) >= const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 >= 180 + const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED*0.5, const.MAX_SPEED)

        elif targetDistance > const.MAPPING_DISTANCE \
                and abs(targetBearing - currentBearing) >= const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 < 180 - const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)
        
        elif targetDistance > const.MAPPING_DISTANCE \
                and abs(targetBearing - currentBearing) >= const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED*0.5)

        elif targetDistance > const.MAPPING_DISTANCE:
            set_velocity(pioneer3at.wheels, const.MAX_SPEED, const.MAX_SPEED)

        else:
            return currentPos, currentBearing, last_scan

def detect_obstacle(robot, hokuyo, width, halfWidth, rangeThreshold,
                    maxRange, braitenbergCoefficients):

    # set obstacle counters
    leftObstacle = 0.0
    rightObstacle = 0.0

    # scan using the Hokuyo
    values = hokuyo.getRangeImage()

    frontObstacle = values[math.floor(halfWidth)]

    # detect obstacle scores based on Braitenberg coefficients
    for k in range(math.floor(halfWidth)):
        if values[k] < rangeThreshold:
            leftObstacle += braitenbergCoefficients[k] \
                            * (1.0 - values[k] / maxRange)

        j = width - k - 1

        if values[j] < rangeThreshold:
            rightObstacle += braitenbergCoefficients[k] \
                                * (1.0 - values[j] / maxRange)

    return leftObstacle, rightObstacle, leftObstacle + rightObstacle, frontObstacle

def prepare_to_map(pioneer3at, targetBearing):

    # Loop for rotating the robot side on with the feature
    while pioneer3at.robot.step(pioneer3at.timestep) != -1:

        # Continually calculate and update robot bearing
        currentBearing = robot_bearing(pioneer3at.imu)

        # Move the robot until side on with the target feature
        # Once side on mark the startingposition of the feature mapping
        if abs(targetBearing - currentBearing) > const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 >= 180 + const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, -0.25*const.MAX_SPEED, 0.25*const.MAX_SPEED)
        elif abs(targetBearing - currentBearing) > const.ANGULAR_THRESHOLD \
                and (targetBearing - currentBearing + 360) % 360 <= 180 - const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, 0.25*const.MAX_SPEED, -0.25*const.MAX_SPEED)
        elif abs(targetBearing - currentBearing) > const.ANGULAR_THRESHOLD:
            set_velocity(pioneer3at.wheels, 0.25*const.MAX_SPEED, -0.25*const.MAX_SPEED)
        else:
            set_velocity(pioneer3at.wheels, 0, 0)
            return

# NTS: need to improve mapping, use front sensor to avoid nothing loop
def lidar_mapping(pioneer3at, target, i, first_scan):

    # Define the mapping optimal mapping distance as a new variable
    threshold = target[2]

    #Unused SLAM variable
    #last_scan = first_scan

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
    scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                            pioneer3at.lidar, currentPos,
                                            currentBearing,
                                            scan='feature', threshold=threshold)

    
    # 4D Transformation method using gps and imu for georeferencing scans
    T = np.eye(4)
    axisRotation = np.deg2rad((180 - currentBearing + 360) % 360) 
    T[:3,:3] = scan.get_rotation_matrix_from_xyz((0,axisRotation, 0))
    T[0,3] = currentPos[0]
    T[1,3] = currentPos[1]
    T[2,3] = currentPos[2]
    transformed_scan = scan.transform(T)
    
    # # SlAM method using scans only for georeferencing
    # transformed_scan = slam.rotate_scan(last_scan, scan)
    # last_scan = transformed_scan

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
        if target_distance(lastScanPos, currentPos) > \
            const.SCAN_THRESHOLD:

            # Capture a new scan, rotate based on first scan, and write to file
            logging.debug("Acquiring LiDAR scan ...")
            scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                                    pioneer3at.lidar, currentPos,
                                                    currentBearing,
                                                    scan='feature', threshold=threshold)

            # 4D Transformation method using gps and imu for georeferencing scans
            T = np.eye(4)
            axisRotation = np.deg2rad((180 - currentBearing + 360) % 360) 
            T[:3,:3] = scan.get_rotation_matrix_from_xyz((0,axisRotation, 0))
            T[0,3] = currentPos[0]
            T[1,3] = currentPos[1]
            T[2,3] = currentPos[2]
            transformed_scan = scan.transform(T)
            
            # # SlAM method using scans only for georeferencing
            # transformed_scan = slam.rotate_scan(last_scan, scan)
            # last_scan = transformed_scan
            
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
        
# NTS: Need to incorperate camera, either move camera to side on, or change prepare to map and lidar sample obtained
# NTS: ET1's trajectory's will play a part in this decision
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
    
    #Unused SLAM variable
    #last_scan = first_scan

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

    if scanBearing == 0:
        [mappingPositions[0].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[1].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[2].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[3].append([bbox[1]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
    elif scanBearing == 90:
        [mappingPositions[0].append([bbox[1]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
        [mappingPositions[1].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[2].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[3].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
    elif scanBearing == 180:
        [mappingPositions[0].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[1].append([bbox[1]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
        [mappingPositions[2].append([bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[3].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
    elif scanBearing == 270:
        [mappingPositions[0].append([bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold]) for i in range(xlength+1)]
        [mappingPositions[1].append([bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i]) for i in range(zlength+1)]
        [mappingPositions[2].append([bbox[1]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold]) for i in range(xlength+1)]
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

        # Capture a new scan, rotate based on first scan, and write to file
        logging.debug("Acquiring LiDAR scan ...")
        scan = clust.capture_lidar_scene(pioneer3at.robot, pioneer3at.timestep,
                                                pioneer3at.lidar, currentPos,
                                                currentBearing,
                                                scan='feature', threshold=threshold)
        
        # # SlAM method using scans only for georeferencing
        # transformed_scan = slam.rotate_scan(last_scan, scan)
        # last_scan = transformed_scan
        
        # 4D Transformation method using gps and imu for georeferencing scans
        T = np.eye(4)
        if scanBearing == 0:
            axisRotation = np.deg2rad(currentBearing + 180) % 360 
        elif scanBearing == 90:
            axisRotation = np.deg2rad(currentBearing) 
        elif scanBearing == 180:
            axisRotation = np.deg2rad(currentBearing + 180) % 360 
        elif scanBearing == 270:
            axisRotation = np.deg2rad(currentBearing) 
        else:
            logging.info("Bearing error encountered whilst mapping feature {}"
                        .format(featureNumber+1))
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
    
        # Detect features/clusters within the transformed lidar scan
        logging.debug("Clustering LiDAR scan ...")
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
            logging.info("Feature requires an additional scan(s) along edge {}"
                        .format(edgeCounter+1))
            bboxFlag=True
            xlength = math.floor((bbox[1]-bbox[0])/const.SCAN_THRESHOLD)
        if math.floor((bbox[3]-bbox[2])/const.SCAN_THRESHOLD) > zlength:
            logging.info("Feature requires an additional scan(s) along edge {}"
                        .format(edgeCounter+1))
            bboxFlag=True
            zlength = math.floor((bbox[3]-bbox[2])/const.SCAN_THRESHOLD)
        
        # If additional scans are required update mapping positions
        if bboxFlag:
            if firstScanBearing == 0:
                mappingPositions[0][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[1][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
                mappingPositions[2][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[3][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
            elif firstScanBearing == 90:
                mappingPositions[0][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
                mappingPositions[1][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[2][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
                mappingPositions[3][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
            elif firstScanBearing == 180:
                mappingPositions[0][:] = [[bbox[1]+threshold, 0, bbox[3]-zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[1][:] = [[bbox[1]-zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[2]-threshold] for i in range(xlength+1)]
                mappingPositions[2][:] = [[bbox[0]-threshold, 0, bbox[2]+zero_division(bbox[3]-bbox[2],zlength)*i] for i in range(zlength+1)]
                mappingPositions[3][:] = [[bbox[0]+zero_division(bbox[1]-bbox[0],xlength)*i, 0, bbox[3]+threshold] for i in range(xlength+1)]
            elif firstScanBearing == 270:
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
        if ((scanBearing == 90 or scanBearing == 270) and scanCounter == xlength+1) or \
            ((scanBearing == 0 or scanBearing == 180) and scanCounter == zlength+1):
            logging.info("Mapping of edge {} feature {} complete"
                        .format(edgeCounter+1, featureNumber+1))
            scanCounter = 0
            edgeCounter +=1
            if edgeCounter == 4:
                logging.info("Mapping of feature {} complete"
                            .format(featureNumber+1))
                mappingFlag = False
            else:    
                currentPos, currentBearing, last_scan = nav_to_point(scanCounter, mappingPositions[edgeCounter][scanCounter], pioneer3at, False, currentPos, first_scan)
                scanBearing = (scanBearing + 270) % 360
                prepare_to_map(pioneer3at, scanBearing)
        else:
            currentPos, currentBearing, last_scan = nav_to_point(scanCounter, mappingPositions[edgeCounter][scanCounter], pioneer3at, False, currentPos, first_scan)
            prepare_to_map(pioneer3at, scanBearing)

def zero_division(n, d):

    return n / d if d else 0

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
