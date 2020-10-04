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

def worker(robot):

    # Function for logging
    t = threading.currentThread()
    gnss_logger = logging.getLogger("GNSS")
    gnss_logger.debug('Starting log for Pioneer3at')
    time.sleep(0.5)

    while getattr(t, "do_run", True):
        with open(os.path.join(const.OUTPUT_PATH, const.COORDINATE_FILENAME), 'a') as outFile:
            robotcoordinate = robot_position(robot.gps)
            robotbearing = robot_bearing(robot.imu)
            if robot.last_feature_points is not None:
                x, y, z = np.array(robot.last_feature_points.points).T

                for coord in zip(x, y, z):
                    lidarpoint = geojson.Feature(geometry=geojson.Point((coord[0], coord[2])),
                                            properties={"type": "lidarfeature",
                                                        "bearing":0.0,
                                                        "NumClusters":0,
                                                        "AvgPointDensity":0.0})
                    outFile.write("\n")
                    geojson.dump(lidarpoint, outFile)
                    robot.last_feature_points = None
                #print(x[0], y[0], z[0])


            if robot.average_density is not None and robot.lidar_num_clusters is not None:
                outFeature = geojson.Feature(geometry=geojson.Point((robotcoordinate[0], -robotcoordinate[2])),
                                            properties={"type": "scan",
                                                        "bearing":round(robotbearing,2),
                                                        "NumClusters":robot.lidar_num_clusters,
                                                        "AvgPointDensity":round(robot.average_density, 2)})
                robot.average_density = None
                robot.lidar_num_clusters = None

            else:
                outFeature = geojson.Feature(geometry=geojson.Point((robotcoordinate[0], -robotcoordinate[2])),
                                            properties={"type": "waypoint",
                                                        "bearing":round(robotbearing,2),
                                                        "NumClusters":0,
                                                        "AvgPointDensity":0.0})

            outFile.write("\n")
            geojson.dump(outFeature, outFile)

        # Reset values


        time.sleep(0.2)
