"""pioneer_py controller."""

from controller import Robot, Lidar, GPS, InertialUnit, Camera, RangeFinder
import csv
import sys
from pyproj import Proj, Transformer
import os
import math

def location_offset(position, x, y, z):

    return (position[0]+x, position[1]+z, position[2]+y)

def difference(target, current):

    return (target[0] - current[0], target[1] - current[1], target[2] - current[2])

def bearing(displacement):

    initial_bearing = math.degrees(math.atan2(displacement[0],displacement[1]))
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def distance(displacement):

    return math.sqrt((displacement[0])**2 + (displacement[1])**2 + (displacement[2])**2)

def elevation(displacement, dist):

    return math.asin(displacement[2] / dist)

# define projection
CRS_FROM = 4326  # WGS 84
CRS_TO = 7855  # GDA2020 Z55
pyproj_transformer = Transformer.from_crs(CRS_FROM, CRS_TO, always_xy=True)

# define home location
HOME = (144.962, -37.7944, 40)
HOME_LOCATION = (*pyproj_transformer.transform(HOME[0], HOME[1]), HOME[2])
TARGET_POSITION = location_offset(HOME_LOCATION, 0, 0, -3) 

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get and enable robot devices
hokuyoFront = robot.getLidar("HokuyoFront")
hokuyoRear = robot.getLidar("HokuyoRear")
hokuyoFront.enable(timestep)
hokuyoFront.enablePointCloud()
hokuyoRear.enable(timestep)
hokuyoRear.enablePointCloud()
lidar = robot.getLidar('Velodyne HDL-32E')
lidar.enable(timestep)
lidar.enablePointCloud()
gps = robot.getGPS('gps')
gps.enable(timestep)
imu = robot.getInertialUnit('imu')
imu.enable(timestep)
cameraRange = robot.getRangeFinder("MultiSenseS21 meta range finder")
cameraRange.enable(timestep)
cameraCenter = robot.getCamera("MultiSenseS21 meta camera")
cameraCenter.enable(timestep)
cameraLeft = robot.getCamera("MultiSenseS21 left camera")
cameraLeft.enable(timestep)
cameraRight = robot.getCamera("MultiSenseS21 right camera")
cameraRight.enable(timestep)
wheels = []
wheelNames = ['front left wheel', 'front right wheel','back left wheel', 'back right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
# start main loop
while robot.step(timestep) != -1:

    # Process sensor data here.
    currentGPSPos = gps.getValues()
    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
    currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
    displacement = difference(currentPos, TARGET_POSITION)
    currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
    bearingToTarget = bearing(displacement)
    DistanceToTarget = distance(displacement)
    ElevationToTarget = elevation(displacement, DistanceToTarget)

    # move based on bearing and distance to target
    if DistanceToTarget < 1:
        print("prepare to map features")
        newBearing = (currentBearing + 90 + 360) % 360
        print("new Bearing:", newBearing, currentBearing)
        while robot.step(timestep) != -1:
            currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
            if abs(newBearing - currentBearing) > 5:
                leftSpeed = -1.0
                rightSpeed = 1.0

                wheels[0].setVelocity(leftSpeed)
                wheels[1].setVelocity(rightSpeed)
                wheels[2].setVelocity(leftSpeed)
                wheels[3].setVelocity(rightSpeed)
            else:
                print("Start feature mapping")
                while robot.step(timestep) != -1: # detect return to starting position
                    leftSpeed = 1.0
                    rightSpeed = 1.0 * 0.5

                    wheels[0].setVelocity(leftSpeed)
                    wheels[1].setVelocity(rightSpeed)
                    wheels[2].setVelocity(leftSpeed)
                    wheels[3].setVelocity(rightSpeed)

                #return home code
                break
        break
        
    elif abs(bearingToTarget - currentBearing) > 1: # update to turn towards correct direction
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:
        leftSpeed = 1.0
        rightSpeed = 1.0

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

