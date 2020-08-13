#!/usr/bin/env python

''' Functions for logging in the robot'''

import logging
import time
from research.navigation import robot_position


def worker(arg, gps_object):
    ''' Function for logging '''
    logging.debug('Starting log for Pioneer3at')
    while not arg['stop']:
        # Log GPS every 0.5 seconds
        logging.debug('GNSS x:{:6f}, y:{:6f}, z:{:6f}'.format(*robot_position(gps_object)))
        time.sleep(0.5)
