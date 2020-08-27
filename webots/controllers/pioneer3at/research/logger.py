#!/usr/bin/env python

''' Functions for logging in the robot'''

import logging
import time
import geojson
import os
from research.navigation import robot_position, robot_bearing
import research.constants as const

def worker(gps_object, imu_object):
    ''' Function for logging '''
    gnss_logger = logging.getLogger("GNSS")
    gnss_logger.debug('Starting log for Pioneer3at')
    time.sleep(0.5)

    while True:
        with open(os.path.join(const.OUTPUT_PATH, const.COORDINATE_FILENAME), 'a') as outFile:
            robotcoordinate = robot_position(gps_object)
            robotbearing = robot_bearing(imu_object)
            outFeature = geojson.Feature(geometry=geojson.Point((robotcoordinate[0], robotcoordinate[2])), properties={"bearing":round(robotbearing,2)})
            #outFeature = geojson.Point((robotcoordinate[0], robotcoordinate[2]))

            outFile.write("\n")
            #print(outFeature)
            geojson.dump(outFeature, outFile)
        time.sleep(0.2)
        # Log GPS every 0.5 seconds
        #gnss_logger.info('Coordinate,{:6f},{:6f},{:6f},GNSS'.format(*robot_position(gps_object)))
        ##time.sleep(0.2)

def write_featurepoints(featurelist, gps_object, imu_object):
    with open(os.path.join(const.OUTPUT_PATH, const.FEATURE_FILENAME), 'a') as outFile:
            robotcoordinate = robot_position(gps_object)
            robotbearing = robot_bearing(imu_object)
            for feature in featurelist:

                outFeature = geojson.Feature(geometry=geojson.Point((feature[0][0],feature[0][2])),
                                             properties={})
                geojson.dump(outFeature, outFile)
                outFile.write("\n")
            outFile.write("\n")
