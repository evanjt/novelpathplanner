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

# start main loop
for i in range(len(TARGET_POSITIONS)):

    robot.step(timestep)
    currentGPSPos = gps.getValues()
    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
    currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
    displacement = difference(currentPos, TARGET_POSITIONS[i])
    currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
    bearingToTarget = bearing(displacement)
    
    while robot.step(timestep) != -1:

        # Process sensor data here.
        currentGPSPos = gps.getValues()
        utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
        currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
        displacement = difference(currentPos, TARGET_POSITIONS[i])
        currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
        DistanceToTarget = distance(displacement)
        # ElevationToTarget = elevation(displacement, DistanceToTarget)
    
        # move based on bearing and distance to target
        if abs(bearingToTarget - currentBearing) > 1 and (bearingToTarget - currentBearing + 360) % 360 > 180:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED * -1.0
        elif abs(bearingToTarget - currentBearing) > 1 and (bearingToTarget - currentBearing + 360) % 360 < 180:
            leftSpeed = MAX_SPEED * -1.0
            rightSpeed = MAX_SPEED
        elif DistanceToTarget > 1 and OBSTACLE:
            # avoid obstacle
            # <insert Evans hokuyo obstacle avoidance code here>
            
            # calculate new bearing to target once obstacle has been avoided
            bearingToTarget = bearing(displacement)
        elif DistanceToTarget > 1 and not OBSTACLE:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED       
        else:
            print("prepare to map features")
            newBearing = (currentBearing + 90 + 360) % 360
            print("new Bearing:", newBearing, currentBearing)
            while robot.step(timestep) != -1:
                currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
                if abs(newBearing - currentBearing) > 1:
                    leftSpeed = MAX_SPEED * -1.0
                    rightSpeed = MAX_SPEED
    
                    wheels[0].setVelocity(leftSpeed)
                    wheels[1].setVelocity(rightSpeed)
                    wheels[2].setVelocity(leftSpeed)
                    wheels[3].setVelocity(rightSpeed)
                else:
                    print("Start feature mapping")
                    currentGPSPos = gps.getValues()
                    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
                    currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
                    startingPos = currentPos
                    hasMoved = False
                    while robot.step(timestep) != -1:
                        # Process sensor data here.
                        currentGPSPos = gps.getValues()
                        utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
                        currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
                        displacement = difference(currentPos, startingPos)
                        DistanceToStart = distance(displacement)
                        
                        if hasMoved and DistanceToStart < 0.05:
                            if len(TARGET_POSITIONS)-1 == i:
                                print("Return home")
                                robot.step(timestep)
                                currentGPSPos = gps.getValues()
                                utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
                                currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
                                displacement = difference(currentPos, HOME_LOCATION)
                                bearingToHome = bearing(displacement)
                                while robot.step(timestep) != -1:
                                    # Process sensor data here.
                                    currentGPSPos = gps.getValues()
                                    utmPos = (*pyproj_transformer.transform(currentGPSPos[0], currentGPSPos[1]), currentGPSPos[2])
                                    currentPos = location_offset(utmPos, -0.15, -0.3, 0) 
                                    displacement = difference(currentPos, HOME_LOCATION)
                                    DistanceToHome = distance(displacement)
                                    currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
                                    if abs(bearingToHome - currentBearing) > 1 and (bearingToHome - currentBearing + 360) % 360 > 180:
                                        leftSpeed = MAX_SPEED
                                        rightSpeed = MAX_SPEED * -1.0
                                    elif abs(bearingToHome - currentBearing) > 1 and (bearingToHome - currentBearing + 360) % 360 < 180:
                                        leftSpeed = MAX_SPEED * -1.0
                                        rightSpeed = MAX_SPEED * 1.0
                                    elif DistanceToTarget > 1 and OBSTACLE:
                                        # avoid obstacle
                                        # <insert Evans hokuyo obstacle avoidance code here>
                                        
                                        # calculate new bearing to target once obstacle has been avoided
                                        bearingToHome = bearing(displacement)
                                    elif DistanceToHome < 1:
                                        print("Survey complete")
                                        leftSpeed = 0
                                        rightSpeed = 0 
                                        wheels[0].setVelocity(leftSpeed)
                                        wheels[1].setVelocity(rightSpeed)
                                        wheels[2].setVelocity(leftSpeed)
                                        wheels[3].setVelocity(rightSpeed)
                                        break
                                    #need to reset bearing to home every few meters to ensure on correct course                                      
                                    else:
                                        leftSpeed = MAX_SPEED
                                        rightSpeed = MAX_SPEED                                       
                                    wheels[0].setVelocity(leftSpeed)
                                    wheels[1].setVelocity(rightSpeed)
                                    wheels[2].setVelocity(leftSpeed)
                                    wheels[3].setVelocity(rightSpeed)
                                break
                            else:
                                break
                        elif not hasMoved and DistanceToStart > 0.05:
                            hasMoved = True
                        else:
                            # instead of speed should map the feature based on range to feature
                            # should also include obstacle detection when feature mapping
                            leftSpeed = MAX_SPEED
                            rightSpeed = MAX_SPEED * 0.5
    
                        wheels[0].setVelocity(leftSpeed)
                        wheels[1].setVelocity(rightSpeed)
                        wheels[2].setVelocity(leftSpeed)
                        wheels[3].setVelocity(rightSpeed)
                    break
            break
    
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)
        wheels[2].setVelocity(leftSpeed)
        wheels[3].setVelocity(rightSpeed)

