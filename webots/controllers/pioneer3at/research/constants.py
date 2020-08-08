import os
from research.navigation import location_offset
## CONSTANTS
# define home location
HOME_LOCATION  =(0, 0, 0)
TARGET_POSITIONS = [location_offset(HOME_LOCATION, 0, 0, 7),
                    location_offset(HOME_LOCATION, 0, 0, -7),
                    location_offset(HOME_LOCATION, 7, 0, 0),
                    location_offset(HOME_LOCATION, -7, 0, 0),
                    HOME_LOCATION]

# define other variables
MAX_SPEED = 5.24
OBSTACLE_THRESHOLD = 0.1
DECREASE_FACTOR = 0.9


OUTPUT_PATH = os.path.abspath(os.path.join(os.getcwd(), \
                              os.pardir,os.pardir, os.pardir, \
                              'output'))
