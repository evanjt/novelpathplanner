#!/usr/bin/python3

'''
Functions for logging in the robot

    Authors:    Josh Clough
                Evan Thomas
'''

import logging
import time
import geojson
import os
import threading
import datetime
import numpy as np

# Project specific functions
from research.navigation import robot_position, robot_bearing
import research.constants as const


def form_line(x, y, target, scan_no = -1, edge_no = -1, pointtype = "waypoint", bearing = 0.0,
              numclusters = 0, avgpointdensity = 0.0):

    if pointtype == 'scan':
        point_threshold = int(avgpointdensity >= const.POINT_DENSITY)
    else:
        point_threshold = -1 # Non existant

    message = geojson.Feature(geometry=geojson.Point((x, y)),
                                 properties={"time": datetime.datetime.now().isoformat(),
                                             "type": pointtype,
                                             "currentTarget": target,
                                             "currentScan": scan_no,
                                             "currentEdge": edge_no,
                                             "bearing": bearing,
                                             "NumClusters": numclusters,
                                             "AvgPointDensity": avgpointdensity,
                                             "MeetsPointThshld": point_threshold})
    return message

def worker(robot):
    log_path = os.path.join(const.OUTPUT_PATH, const.COORDINATE_FILENAME)

    if os.path.exists(log_path):
        logging.debug("Removing coordinate file: {}".format(log_path))
        os.remove(log_path)

    # Function for logging
    t = threading.currentThread()
    gnss_logger = logging.getLogger("GNSS")
    gnss_logger.debug('Starting log for Pioneer3at')
    time.sleep(0.5)

    while getattr(t, "do_run", True):
        with open(log_path, 'a') as outFile:
            robotcoordinate = robot_position(robot.gps)
            robotbearing = (180 - robot_bearing(robot.imu) + 180 ) % 360

            if robot.average_density is not None and robot.lidar_num_clusters is not None:
                if const.DEVICE == 'lidar': # LiDAR mode has no edges
                    edge_no = -1
                    scan_type = "scan_lidar"
                elif const.DEVICE == 'camera':
                    edge_no = robot.edge_counter
                    scan_type = "scan_camera"

                outFeature = form_line(x = robotcoordinate[0], y = robotcoordinate[2],
                                       pointtype = scan_type, bearing = round(robotbearing,2),
                                       numclusters = robot.lidar_num_clusters,
                                       avgpointdensity = round(robot.average_density, 2),
                                       target = robot.current_target,
                                       scan_no = robot.scan_counter, edge_no = edge_no)

                robot.average_density = None
                robot.lidar_num_clusters = None
            if robot.warning is not None: # Print warning message to log
                outFeature = form_line(x = robotcoordinate[0], y = robotcoordinate[2],
                                       pointtype = str("warningMSG-" + robot.warning), bearing = round(robotbearing,2),
                                       target = robot.current_target)
                robot.warning = None # Return to no warning

            else:

                outFeature = form_line(x = robotcoordinate[0], y = robotcoordinate[2],
                                       pointtype = "waypoint", bearing = round(robotbearing,2),
                                       target = robot.current_target)

            outFile.write("\n")
            geojson.dump(outFeature, outFile)

        # Reset values


        time.sleep(0.2)
