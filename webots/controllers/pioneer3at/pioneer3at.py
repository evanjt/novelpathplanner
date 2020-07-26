"""pioneer3at controller."""

from controller import Robot, Lidar, GPS, InertialUnit, Camera, RangeFinder
import csv
import sys
from pyproj import Proj, Transformer
import os
import math

def set_velocity(wheels, leftSpeed, rightSpeed):

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

def robot_position(gps):

    currentGPSPos = gps.getValues()
    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
    currentPos = location_offset(utmPos, -0.15, -0.3, 0)
    
    return currentPos
      
def robot_bearing(imu):

    return (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360

def target_bearing(current, target):

    displacement = difference(current, target)
    bearingToTarget = bearing(displacement)
    
    return bearingToTarget

def target_distance(current, target):

    displacement = difference(current, target)
    distanceToTarget = distance(displacement)
    
    return distanceToTarget

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
TARGET_POSITIONS = [location_offset(HOME_LOCATION, 0, 0, -3), 
                    location_offset(HOME_LOCATION, 0, 0, 3)]

# define other variables
MAX_SPEED = 5.24
OBSTACLE = False

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

print("Beggining survey of the %.d provided features" %len(TARGET_POSITIONS))

# Loop through the target features provided
for i in range(len(TARGET_POSITIONS)):

    # Calculate initial bearing to target feature
    robot.step(timestep)
    currentPos = robot_position(gps)
    targetBearing = target_bearing(currentPos, TARGET_POSITIONS[i]) # need to reset bearing to feature every few meters to account for error in initial course
    
    # Loop for robot navigation to the feature
    while robot.step(timestep) != -1:
    
        # Continually calculate and update robot position, bearing and distance to target feature
        currentPos = robot_position(gps)
        currentBearing = robot_bearing(imu)
        targetDistance = target_distance(currentPos, TARGET_POSITIONS[i])     
    
        # Move the robot based on the bearing and distance to the target feature
        # Once within range set the target bearing to plus 90 degrees ready for mapping
        if abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 > 180:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED * -1.0
            
        elif abs(targetBearing - currentBearing) > 1 and (targetBearing - currentBearing + 360) % 360 < 180:
            leftSpeed = MAX_SPEED * -1.0
            rightSpeed = MAX_SPEED
            
        elif targetDistance > 1 and OBSTACLE:
            # avoid obstacle
            # <insert Evans hokuyo obstacle avoidance code here>
            # calculate and set new bearing to target once obstacle has been fully avoided
            # reset OBSTACLE boolean flag
            targetBearing = target_bearing(currentPos, TARGET_POSITIONS[i])
            
        elif targetDistance > 1 and not OBSTACLE:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED  
                 
        else:
            newBearing = (currentBearing + 90 + 360) % 360
            
            # Loop for rotating the robot side on with the feature
            while robot.step(timestep) != -1:
            
                # Continually calculate and update robot bearing
                currentBearing = robot_bearing(imu)
                
                # Move the robot until side on with the target feature
                # Once side on mark the startingposition of the feature mapping
                if abs(newBearing - currentBearing) > 1:
                    leftSpeed = MAX_SPEED * -1.0
                    rightSpeed = MAX_SPEED
                    set_velocity(wheels, leftSpeed, rightSpeed)
                
                else:
                    currentPos = robot_position(gps)
                    startingPos = currentPos
                    hasMoved = False
                    
                    # Loop for feature mapping
                    while robot.step(timestep) != -1:
                        
                        # Continually calculate and update robot position and distance to feature mapping start point
                        currentPos = robot_position(gps)
                        distanceToStart = target_distance(currentPos, startingPos)
                        
                        # Navigate the robot around the feature until it returns to its starting point
                        if hasMoved and distanceToStart < 0.05:
                        
                            # Once returned to the starting point
                            # either return home or break the loop to continue mapping the next feature
                            if len(TARGET_POSITIONS)-1 == i:
                            
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
                                        leftSpeed = MAX_SPEED
                                        rightSpeed = MAX_SPEED * -1.0
                                        
                                    elif abs(bearingToHome - currentBearing) > 1 and (bearingToHome - currentBearing + 360) % 360 < 180:
                                        leftSpeed = MAX_SPEED * -1.0
                                        rightSpeed = MAX_SPEED * 1.0
                                        
                                    elif distanceToHome > 1 and OBSTACLE:
                                        # avoid obstacle
                                        # <insert Evans hokuyo obstacle avoidance code here>
                                        # calculate new bearing to target once obstacle has been avoided
                                        bearingToHome = target_bearing(currentPos, HOME_LOCATION)
                                        
                                    elif distanceToHome < 1:
                                        print("Survey complete")
                                        leftSpeed = 0
                                        rightSpeed = 0 
                                        set_velocity(wheels, leftSpeed, rightSpeed)
                                        break
                                                                            
                                    else:
                                        leftSpeed = MAX_SPEED
                                        rightSpeed = MAX_SPEED 
                                                                              
                                    set_velocity(wheels, leftSpeed, rightSpeed)
                                    
                                break
                            
                            else:
                                break
                                
                        elif not hasMoved and distanceToStart > 0.05:
                            hasMoved = True
                            
                        else:
                            # instead of speed should map the feature based on range to feature
                            # <insert feature mapping code here>
                            # should also incorperate obstacle detection code into feature mapping
                            leftSpeed = MAX_SPEED
                            rightSpeed = MAX_SPEED * 0.5
    
                        set_velocity(wheels, leftSpeed, rightSpeed)
                        
                    break
            break
    
        set_velocity(wheels, leftSpeed, rightSpeed)

