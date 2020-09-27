#!/usr/bin/python3

'''
    Constants required for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import os

# Define home location
HOME_LOCATION = [0, 0, 0]

# Define robot parameters 
MAX_SPEED = 5.24
VERTICAL_VOF = 45
SCANNER_HEIGHT = 0.5

# Define the mapping mode
DEVICE = 'camera'

# Define the required proximity (m) from a target point to be a successful navigation
MAPPING_DISTANCE = 1

# Define feature detection threshold size (m)
FEATURE_THRESHOLD = 1

# Define threshold distance (m) between taking lidar scans whilst mapping
SCAN_THRESHOLD = 2

# Define threshold distance (m) between updating the bearing to target whilst navigating
# When in SLAM mode this is also the threshold distance (m) between taking lidar scans whilst navigating
MOVEMENT_THRESHOLD = 1

# Define threshold angle (degrees) for course corrections to the target whilst navigating
ANGULAR_THRESHOLD = 1

# Define threshold distance (m) that needs to be reached before and feature can be identified as fully mapped
LOOP_THRESHOLD = 2

# Define obstacle detection variables including:
# -threshold distance before avoidance as a per the braitenburg coeffcient, 
# -buffer distance to avoid constant correction as a per the braitenburg coeffcient,
# -speed decrease factor once an obstacle has been detected
OBSTACLE_THRESHOLD = 0.1
OBSTACLE_BUFFER = 0.05
DECREASE_FACTOR = 0.9

# Define the output filenames and paths
COORDINATE_FILENAME = 'coordinates.json'
FEATURE_FILENAME = 'featurepoints.json'
OUTPUT_PATH = os.path.abspath(os.path.join(os.getcwd(),
                                           os.pardir,
                                           os.pardir,
                                           os.pardir,
                                           'output'))
LOGFILENAME = 'logfile.txt'