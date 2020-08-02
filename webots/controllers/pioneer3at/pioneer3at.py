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

def prepare_to_map(robot, timestep, imu, newBearing):

    # Loop for rotating the robot side on with the feature
    while robot.step(timestep) != -1:
    
        # Continually calculate and update robot bearing
        currentBearing = robot_bearing(imu)
        
        # Move the robot until side on with the target feature
        # Once side on mark the startingposition of the feature mapping
        if abs(newBearing - currentBearing) > 1:
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
    front = 540
    # front = round(width/2)
    frontSide = 360
    side = 180

    # Loop for feature mapping
    while robot.step(timestep) != -1:
        
        # Continually calculate and update robot position and distance to feature mapping start point
        currentPos = robot_position(gps)
        distanceToStart = target_distance(currentPos, startingPos)
        values = hokuyo.getRangeImage()
        
        print(values[front], values[frontSide], values[side])
        # print(getDistance(sensors[0]), getDistance(sensors[1]))
        
        # Navigate the robot around the feature until it returns to its starting point
        if hasMoved and distanceToStart < 0.5:
            return
                
        elif not hasMoved and distanceToStart > 0.5:
            hasMoved = True
            
        else:
            if values[side] < threshold or values[frontSide] < threshold+0.9:
                set_velocity(wheels, MAX_SPEED, MAX_SPEED*0.6)
        
            elif values[side] > threshold+0.1 or values[frontSide] > threshold+1:
                set_velocity(wheels, MAX_SPEED*0.6, MAX_SPEED)
        
            else:
                set_velocity(wheels, MAX_SPEED, MAX_SPEED)

def return_home(robot, timestep, wheels, gps, imu):

    # Calculate robot position and bearing to home
    robot.step(timestep)
    currentPos = robot_position(gps)
    bearingToHome = target_bearing(currentPos, HOME_LOCATION) # need to reset bearing to home every few meters to account for error in initial course 

    # Loop for returning home once the last feature has been mapped
    while robot.step(timestep) != -1:
        
        # Calculate robot position and distance to home
        currentPos = robot_position(gps)
        currentBearing = robot_bearing(imu)
        distanceToHome = target_distance(currentPos, HOME_LOCATION)
        
        # Move the robot based on the bearing and distance to the home
        # Once within home range stop the robot and exit the controller
        if abs(bearingToHome - currentBearing) > 1 and (bearingToHome - currentBearing + 360) % 360 > 180:
            set_velocity(wheels, -MAX_SPEED, MAX_SPEED)
            
        elif abs(bearingToHome - currentBearing) > 1 and (bearingToHome - currentBearing + 360) % 360 < 180:
            set_velocity(wheels, MAX_SPEED, -MAX_SPEED)
            
        elif distanceToHome > 1 and OBSTACLE:
            # avoid obstacle
            # <insert Evans hokuyo obstacle avoidance code here>
            # calculate new bearing to target once obstacle has been avoided
            bearingToHome = target_bearing(currentPos, HOME_LOCATION)
            
        elif distanceToHome < 1:
            print("Survey complete")
            set_velocity(wheels, 0, 0)
            return
                                                
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

def set_velocity2(wheels, leftSpeed, rightSpeed):

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(BACK_SLOWDOWN * leftSpeed)
    wheels[3].setVelocity(BACK_SLOWDOWN * rightSpeed)

def robot_position(gps):

    currentGPSPos = gps.getValues()
    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
    currentPos = location_offset(utmPos, 0, 0.2, 0.2)
    
    return currentPos
      
def robot_bearing(imu):

    return (math.degrees(imu.getRollPitchYaw()[2]) * -1 + 360) % 360

def target_bearing(current, target):

    displacement = difference(current, target)
    bearingToTarget = bearing(displacement)
    
    return bearingToTarget

def target_distance(current, target):

    displacement = difference(current, target)
    distanceToTarget = distance(displacement)
    
    return distanceToTarget

def location_offset(position, x, y, z):

    return (position[0]-x, position[1]-z, position[2]-y)


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
TARGET_POSITIONS = [location_offset(HOME_LOCATION, 0, 0, 7), 
                    location_offset(HOME_LOCATION, 0, 0, -7),
                    location_offset(HOME_LOCATION, -7, 0, 0),
                    location_offset(HOME_LOCATION, 7, 0, 0)]

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

print("Beggining survey of the %.d provided features" %len(TARGET_POSITIONS))

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
        
        # Once within range map the feature
        if targetDistance > 2 and obstacle[2] > OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - DECREASE_FACTOR * obstacle[2]) * MAX_SPEED / obstacle[2]
            set_velocity2(wheels, speed_factor * obstacle[0], speed_factor * obstacle[1])
            flag = False
            
        elif obstacle[2] < OBSTACLE_THRESHOLD and flag == False:
            targetBearing = target_bearing(currentPos, TARGET_POSITIONS[i])
            flag = True
        
        elif abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 > 180:
            set_velocity(wheels, MAX_SPEED*0.5, MAX_SPEED)
            
        elif abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 < 180:
            set_velocity(wheels, MAX_SPEED, MAX_SPEED*0.5)
            
        elif targetDistance > 2 and obstacle[2] < OBSTACLE_THRESHOLD:
            set_velocity(wheels, MAX_SPEED, MAX_SPEED)  
                 
        else:
            prepare_to_map(robot, timestep, imu, (currentBearing + 90 + 360) % 360)
            feature_mapping(robot, timestep, wheels, gps, hokuyoFront, hkfWidth, 1.5)
            break

# Once the survey is complete return home           
return_home(robot, timestep, wheels, gps, imu)
