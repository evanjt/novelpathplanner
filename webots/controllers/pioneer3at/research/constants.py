#!/usr/bin/python3
'''
    Constants required for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import os


# define home location
HOME_LOCATION = [0, 0, 0]

# define other variables
MAX_SPEED = 5.24
VERTICAL_VOF = 45
SCANNER_HEIGHT = 0.5
FEATURE_THRESHOLD = 2
MAPPING_THRESHOLD = 0.5
SCAN_THRESHOLD = 10
MOVEMENT_THRESHOLD = 0.5
ANGULAR_THRESHOLD = 1
LOOP_THRESHOLD = 2
OBSTACLE_THRESHOLD = 0.1
OBSTACLE_BUFFER = 0.05
DECREASE_FACTOR = 0.9
DEVICE = 'camera'
COORDINATE_FILENAME = 'coordinates.json'
FEATURE_FILENAME = 'featurepoints.json'
MAPPING_DISTANCE = 2

OUTPUT_PATH = os.path.abspath(os.path.join(os.getcwd(),
                                           os.pardir,
                                           os.pardir,
                                           os.pardir,
                                           'output'))

LOGFILENAME = 'logfile.txt'

