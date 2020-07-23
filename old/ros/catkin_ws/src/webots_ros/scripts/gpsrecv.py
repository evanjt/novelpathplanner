#!/usr/bin/env python

import rospy
#import pykalman # maybe for later
#import numpy as np
from sensor_msgs.msg import NavSatFix
from pyproj import CRS, Transformer

ORIGIN_CRS = 4326
TARGET_CRS = 28355 # GDA2020 Z55
HEADER_RPT_INTERVAL = 10

def gps_header():
    print()
    print("Time\t\tLongitude\tLatitude\tAltitude\tUTM_X\t\tUTM_Y")
    print("-------------------------------------------------------------------------------------")


if __name__ == '__main__':
    rospy.init_node('gpsrecv', anonymous=True)

    current_second = 0
    count = 0

    # Setup transformer for DEG -> UTM
    transformer = Transformer.from_crs(ORIGIN_CRS, TARGET_CRS, always_xy=True)

    # Just print every second, using the time provided by the GPS
    while True:
        data = rospy.wait_for_message('/husky/gps/values', NavSatFix)
        if data.header.stamp.secs > current_second:
            if (count % HEADER_RPT_INTERVAL == 0):
                gps_header()
            current_second = data.header.stamp.secs
            coordinates_DEG = (data.latitude, data.longitude) # Lat/Lon seem to be switched around
            altitude = data.altitude
            coordinates_UTM = transformer.transform(*coordinates_DEG)
            print("{}\t{:.5f}\t{:.5f}\t{:.5f}\t{:.4f}\t{:.4f}".format(data.header.stamp.secs,
                                                                      *coordinates_DEG,
                                                                      altitude,
                                                                      *coordinates_UTM))
            count += 1


