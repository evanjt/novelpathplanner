#!/usr/bin/python3

'''
    Constants required for the use of the pioneer3at
    controller in webots.

    Authors:    Josh Clough
                Evan Thomas
'''

import os
import math
import numpy as np

# Define home location
HOME_LOCATION = [0, 0, 0]

# Define robot parameters
MAX_SPEED = 5.24

# LiDAR parameters
LIDAR_VERTICAL_VOF = 90
LIDAR_HORIZONTAL_FOV = 360
LIDAR_VERTICAL_RESOLUTION = 180
LIDAR_HORIZONTAL_RESOLUTION = 720
SCANNER_HEIGHT = 0.6

# Camera parameters
CAMERA_HORIZONTAL_FOV = 66
CAMERA_VERTICAL_RESOLUTION = 488
CAMERA_HORIZONTAL_RESOLUTION = 648
CAMERA_VERTICAL_VOF = math.degrees(2 * math.atan(math.tan(math.radians(CAMERA_HORIZONTAL_FOV) * 0.5) * (CAMERA_VERTICAL_RESOLUTION / CAMERA_HORIZONTAL_RESOLUTION)))
CAMERA_HEIGHT = 0.3
CAMERA_OFFSET = 0.2
# PIXEL_SIZE = ?
# PIXEL_RESOLUTION = 0.06

# Required point density per square meter
POINT_DENSITY = 250

# Define the mapping mode
DEVICE = 'lidar'
#DEVICE = 'camera'

# Define the required proximity (m) from a target point to be a successful navigation
MAPPING_DISTANCE = 1

# Deinfe the required buffer range (m) for mapping to occur
MAPPING_BUFFER = 0.5

# Define feature detection threshold size (m)
FEATURE_THRESHOLD = 1

# Define threshold distance (m) between taking lidar scans whilst mapping
SCAN_THRESHOLD = 3

# Define threshold distance (m) between updating the bearing to target whilst navigating
MOVEMENT_THRESHOLD = 1

# Define threshold angle (degrees) for course corrections to the target whilst navigating
ANGULAR_THRESHOLD = 1

# Define threshold distance (m) that needs to be reached before and feature can be identified as fully mapped
LOOP_THRESHOLD = 2

# Define obstacle detection variables including:
# -threshold distance before avoidance as a per the braitenburg coeffcient,
# -buffer distance to avoid constant correction as a per the braitenburg coeffcient,
# -speed decrease factor once an obstacle has been detected
OBSTACLE_THRESHOLD = 0.2
OBSTACLE_BUFFER = 0.05
DECREASE_FACTOR = 0.9
OBSTACLE_END_LIMIT = 0.08

# Define trajectory parameters
ANGULAR_NAV_THRESHOLD = 0.0001
TRAJPLANNING_BEARING_OFFSET = 90

# Pure Pursuit constants
PP_K = 0.01 # look forward gain
PP_LFC = 0.6  # [m] look-ahead distance
PP_KP = 0.5  # speed proportional gain
PP_WB = 0.2  # [m] wheel base of vehicle
PP_WEIGHT = 15
PP_SHOW_ANIMATION = False
PP_SHOW_VELOCITY_ANIMATION = False

# Move to pose line fitting constants
MTP_KP_RHO = 2 # 9
MTP_KP_ALPHA = 9 # 15
MTP_KP_BETA = -3 # -3
MTP_DT = 0.1 # 0.01
MTP_PLOT = False # Plot the trajectory to screen

# Eta^3 curve's theta origin is quite different to our world layout, correct with offset
ETA_OFFSET = np.radians(-180)

# Define the output filenames and paths
COORDINATE_FILENAME = 'coordinates.json'
FEATURE_FILENAME = 'featurepoints.json'
OUTPUT_PATH = os.path.abspath(os.path.join(os.getcwd(),
                                           os.pardir,
                                           os.pardir,
                                           os.pardir,
                                           'output'))
LOGFILENAME = 'logfile.txt'
