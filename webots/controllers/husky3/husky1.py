""" Controller for ENGR90037 Husky navigation
    Authors: Evan Thomas evant1@student.unimelb.edu.au
             Josh Clough cloughj@student.unimelb.edu.au

    To be executed as a controller within a Webots simulator
"""

from controller import Robot, Lidar, InertialUnit
import csv
import sys
from math import sqrt


def euclidean_dist(x2, y2, z2):
    """
    Helper to calc 3D euclidean distance. Assumes first coordinate (0,0,0)
    which is ideal for relative LiDAR positions.
    """

    x1, y1, z1 = (0, 0, 0)
    return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Import onboard devices
velodyne = robot.getLidar("Velodyne HDL32E")  # Velodyne LiDAR
imu = robot.getInertialUnit("SwiftNav IMU")  # IMU
gnss = robot.getGPS("SwiftNav RTK")

# Turn the Velodyne LiDAR on, print some status msgs
velodyne.enable(timestep)
velodyne.enablePointCloud()
print("LIDAR\tPoint cloud enabled: {}".format(velodyne.isPointCloudEnabled()))
print("LIDAR\tNumer of layers: {}".format(velodyne.getNumberOfLayers()))
print("LIDAR\tHorizontal resolution: {}".format(velodyne.getHorizontalResolution()))
print("LIDAR\tFrequency: {}".format(velodyne.getFrequency()))

# print(velodyne.getNumberOfPoints())
# print(velodyne.getPointCloud())
print("-----")
# Enable IMU
imu.enable(timestep)
print("IMU\tSampling period: {}".format(imu.getSamplingPeriod()))

print("-----")

# Enable GNSS
gnss.enable(timestep)
print("GNSS\tsampling period: {}".format(gnss.getSamplingPeriod()))

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:

    with open('lidar_one_pulse.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        for layerID in range(0, 32, 6):
            for row in velodyne.getLayerPointCloud(layerID):
                if row.y > -1.0 and row.x != 0 and row.y != 0 and row.z != 0:
                    dist = round(euclidean_dist(row.x, row.y, row.z), 2)
                    if dist < 2.0:
                        writer.writerow([row.layer_id, round(row.x, 2), round(row.y, 2),
                                         round(row.z, 2), round(row.time,2), dist])

    # print(velodyne.getLayerPointCloud(1)[0].x)
    # print(imu.getRollPitchYaw())
    # pointcloud = velodyne.getPointCloud()

    # print(velodyne.getPointCloud()[0].x,
    # velodyne.getPointCloud()[0].y,
    # velodyne.getPointCloud()[0].z,
    # velodyne.getPointCloud()[0].layer_id,
    # velodyne.getPointCloud()[0].time)

    # Print all LiDAR points. Careful...
    # for point in pointcloud:
        # print(point.layer_id)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
# Enter here exit cleanup code.
