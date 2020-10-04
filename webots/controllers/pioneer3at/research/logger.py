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

# Project specific functions
from research.navigation import robot_position, robot_bearing
import research.constants as const

def worker(gps_object, imu_object):
    
    # Function for logging
    t = threading.currentThread()
    gnss_logger = logging.getLogger("GNSS")
    gnss_logger.debug('Starting log for Pioneer3at')
    time.sleep(0.5)

    while getattr(t, "do_run", True):
        with open(os.path.join(const.OUTPUT_PATH, const.COORDINATE_FILENAME), 'a') as outFile:
            robotcoordinate = robot_position(gps_object)
            robotbearing = robot_bearing(imu_object)
            outFeature = geojson.Feature(geometry=geojson.Point((robotcoordinate[0], robotcoordinate[2])), properties={"bearing":round(robotbearing,2)})

            outFile.write("\n")
            geojson.dump(outFeature, outFile)
        time.sleep(0.2)