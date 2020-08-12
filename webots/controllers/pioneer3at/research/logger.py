#!/usr/bin/env python

''' Functions for logging in the robot'''

import logging
import time


def worker(arg):
    ''' Function for logging '''
    #while not arg['stop']:
    logging.debug('Hi from myfunc')
        #time.sleep(0.5)
