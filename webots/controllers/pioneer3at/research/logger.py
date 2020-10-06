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
import numpy as np

# Project specific functions
from research.navigation import robot_position, robot_bearing
import research.constants as const


def form_line(x, y, target, pointtype = "waypoint", bearing = 0.0,
              numclusters = 0, avgpointdensity = 0.0):

    if pointtype == 'scan':
        point_threshold = int(avgpointdensity >= const.POINT_DENSITY)
    else:
        point_threshold = -1 # Non existant

    message = geojson.Feature(geometry=geojson.Point((x, y)),
                                 properties={"type": pointtype,
                                             "currentTarget": target,
                                             "bearing": bearing,
                                             "NumClusters": numclusters,
                                             "AvgPointDensity": avgpointdensity,
                                             "MeetsPointThshld": point_threshold})
    return message

def worker(robot):

    # Function for logging
    t = threading.currentThread()
    gnss_logger = logging.getLogger("GNSS")
    gnss_logger.debug('Starting log for Pioneer3at')
    time.sleep(0.5)

    while getattr(t, "do_run", True):
        with open(os.path.join(const.OUTPUT_PATH, const.COORDINATE_FILENAME), 'a') as outFile:
            robotcoordinate = robot_position(robot.gps)
            robotbearing = (180 - robot_bearing(robot.imu) + 180 ) % 360

            if robot.average_density is not None and robot.lidar_num_clusters is not None:
                outFeature = form_line(x = robotcoordinate[0], y = robotcoordinate[2],
                                       pointtype = "scan", bearing = round(robotbearing,2),
                                       numclusters = robot.lidar_num_clusters,
                                       avgpointdensity = round(robot.average_density, 2),
                                       target = robot.current_target)

                robot.average_density = None
                robot.lidar_num_clusters = None

            else:

                outFeature = form_line(x = robotcoordinate[0], y = robotcoordinate[2],
                                       pointtype = "waypoint", bearing = round(robotbearing,2),
                                       target = robot.current_target)

            outFile.write("\n")
            geojson.dump(outFeature, outFile)

        # Reset values


        time.sleep(0.2)
