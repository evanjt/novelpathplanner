"""pioneer3at controller."""

from controller import Robot, Lidar, GPS, InertialUnit, Camera, RangeFinder, DistanceSensor
import csv
import sys
from pyproj import Proj, Transformer
import os
import math

def detect_obstacle(robot, hokuyo, width, halfWidth, rangeThreshold, maxRange,  braitenbergCoefficients):

    # set obstacle counters
    leftObstacle = 0.0
    rightObstacle = 0.0
    
    values = hokuyo.getRangeImage()
    
    for k in range(math.floor(halfWidth)):

        if values[k] < rangeThreshold:
            leftObstacle += braitenbergCoefficients[k] * (1.0 - values[k] / maxRange)
            
        j = width - k - 1;
        
        if values[j] < rangeThreshold:
            rightObstacle += braitenbergCoefficients[k] * (1.0 - values[j] / maxRange)
            
    return leftObstacle, rightObstacle, leftObstacle + rightObstacle

def prepare_to_map(robot, timestep, imu, targetBearing):

    # Loop for rotating the robot side on with the feature
    while robot.step(timestep) != -1:
    
        # Continually calculate and update robot bearing
        currentBearing = robot_bearing(imu)
        
        # Move the robot until side on with the target feature
        # Once side on mark the startingposition of the feature mapping
        if abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(wheels, -MAX_SPEED, MAX_SPEED)
            
        elif abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(wheels, MAX_SPEED, -MAX_SPEED)
        
        else:
            return

def feature_mapping(robot, timestep, wheels, gps, hokuyo, width, threshold):

    # alter threshold for sonar offset
    threshold = threshold
    
    # Calculate starting position
    robot.step(timestep)
    currentPos = robot_position(gps)
    startingPos = currentPos
    hasMoved = False
    front = round(width/2)
    frontSide = round(width/3)
    side = round(width/6)

    # Loop for feature mapping
    while robot.step(timestep) != -1:
        
        # Continually calculate and update robot position and distance to feature mapping start point
        currentPos = robot_position(gps)
        distanceToStart = target_distance(currentPos, startingPos)
        values = hokuyo.getRangeImage()
        
        # Navigate the robot around the feature until it returns to its starting point
        if hasMoved and distanceToStart < 1:
            return
                
        elif not hasMoved and distanceToStart > 1:
            hasMoved = True
            
        else:
            if values[side] < threshold or values[frontSide] < threshold+0.9:
                set_velocity(wheels, MAX_SPEED, MAX_SPEED*0.6)
        
            elif values[side] > threshold+0.1 or values[frontSide] > threshold+1:
                set_velocity(wheels, MAX_SPEED*0.6, MAX_SPEED)
        
            else:
                set_velocity(wheels, MAX_SPEED, MAX_SPEED)
    
def getBraitenberg(robot, width, halfWidth):

    braitenbergCoefficients = []
    
    for i in range(math.floor(width)):
        braitenbergCoefficients.append(gaussian(i, halfWidth, width / 5))
        
    return braitenbergCoefficients

def gaussian(x, mu, sigma):
    
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) * math.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))

def set_velocity(wheels, leftSpeed, rightSpeed):

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

def robot_position(gps):

    currentGPSPos = gps.getValues()
    # utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
    currentPos = location_offset(currentGPSPos, 0, -0.3, -0.2)
    
    return currentPos
      
def robot_bearing(imu):

    return (math.degrees(imu.getRollPitchYaw()[2]) *-1 + 360) % 360

def target_bearing(current, target):

    displacement = difference(current, target)
    bearingToTarget = bearing(displacement)
    
    return bearingToTarget

def target_distance(current, target):

    displacement = difference(current, target)
    distanceToTarget = distance(displacement)
    
    return distanceToTarget

def location_offset(position, x, y, z):

    return (position[0]+x, position[1]+y, position[2]+z)


def difference(target, current):

    return (target[0] - current[0], target[1] - current[1], target[2] - current[2])

def bearing(displacement):

    initial_bearing = math.degrees(math.atan2(displacement[0],displacement[2]))
    compass_bearing = (initial_bearing *-1 + 180) % 360

    return compass_bearing

def distance(displacement):

    return math.sqrt((displacement[0])**2 + (displacement[1])**2 + (displacement[2])**2)

def elevation(displacement, dist):

    return math.asin(displacement[2] / dist)

# define projection
# CRS_FROM = 4326  # WGS 84
# CRS_TO = 7855  # GDA2020 Z55
# pyproj_transformer = Transformer.from_crs(CRS_FROM, CRS_TO, always_xy=True)

# define home location
# HOME = (144.962, -37.7944, 40)
# HOME_LOCATION = (*pyproj_transformer.transform(HOME[0], HOME[1]), HOME[2])
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
BACK_SLOWDOWN = 0.9

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get and enable robot devices
hokuyoFront = robot.getLidar("HokuyoFront")
hokuyoFront.enable(timestep)
hokuyoFront.enablePointCloud()
hkfWidth = hokuyoFront.getHorizontalResolution()
hkfHalfWidth = hkfWidth / 2.0
hkfMaxRange = hokuyoFront.getMaxRange() # 30
hkfRangeThreshold = hkfMaxRange / 20.0 #1.5
hkfValues = []

# hokuyoRear = robot.getLidar("HokuyoRear")
# hokuyoRear.enable(timestep)
# hokuyoRear.enablePointCloud()

# lidar = robot.getLidar('Velodyne HDL-32E')
# lidar.enable(timestep)
# lidar.enablePointCloud()

gps = robot.getGPS('gps')
gps.enable(timestep)

imu = robot.getInertialUnit('imu')
imu.enable(timestep)

# cameraRange = robot.getRangeFinder("MultiSenseS21 meta range finder")
# cameraRange.enable(timestep)
# cameraCenter = robot.getCamera("MultiSenseS21 meta camera")
# cameraCenter.enable(timestep)
# cameraLeft = robot.getCamera("MultiSenseS21 left camera")
# cameraLeft.enable(timestep)
# cameraRight = robot.getCamera("MultiSenseS21 right camera")
# cameraRight.enable(timestep)
    
wheels = []
wheelNames = ['front left wheel', 'front right wheel','back left wheel', 'back right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

# set the Braitenberg coefficient
hkfBraitenbergCoefficients = getBraitenberg(robot, hkfWidth, hkfHalfWidth)

print("Beggining survey of the %.d provided features" %(len(TARGET_POSITIONS)-1))

# Loop through the target features provided
for i in range(len(TARGET_POSITIONS)):

    # Calculate initial bearing to target feature
    robot.step(timestep)
    currentPos = robot_position(gps)
    targetBearing = target_bearing(currentPos, TARGET_POSITIONS[i]) # need to reset bearing to feature every few meters to account for error in initial course
    flag = False
    
    # Navigate robot to the feature
    while robot.step(timestep) != -1:
    
        # Continually calculate and update robot position, bearing and distance to target feature
        currentPos = robot_position(gps)
        currentBearing = robot_bearing(imu)
        targetDistance = target_distance(currentPos, TARGET_POSITIONS[i])

        # Continually detect obstacles
        obstacle = detect_obstacle(robot, hokuyoFront, hkfWidth, hkfHalfWidth, hkfRangeThreshold, hkfMaxRange,  hkfBraitenbergCoefficients)
        
        # Once within range map the feature stop once returned home
        if targetDistance > 3 and obstacle[2] > OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - DECREASE_FACTOR * obstacle[2]) * MAX_SPEED / obstacle[2]
            set_velocity(wheels, speed_factor * obstacle[0], speed_factor * obstacle[1])
            
        elif targetDistance > 3 and obstacle[2] > OBSTACLE_THRESHOLD-0.05:
            set_velocity(wheels, MAX_SPEED, MAX_SPEED)
            flag = False
        
        elif flag == False:
            targetBearing = target_bearing(currentPos, TARGET_POSITIONS[i])
            flag = True
        
        elif targetDistance > 3 and abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(wheels, MAX_SPEED*0.5, MAX_SPEED)
            
        elif targetDistance > 3 and abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(wheels, MAX_SPEED, MAX_SPEED*0.5)
            
        elif targetDistance > 3:
            set_velocity(wheels, MAX_SPEED, MAX_SPEED)
            
        elif i == len(TARGET_POSITIONS)-1:
            print("Survey complete")
            set_velocity(wheels, 0, 0)
            break  
                 
        else:
            prepare_to_map(robot, timestep, imu, (currentBearing + 90) % 360)
            feature_mapping(robot, timestep, wheels, gps, hokuyoFront, hkfWidth, 1.5)
            break
