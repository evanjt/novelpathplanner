"""pioneer_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Lidar, GPS, InertialUnit, Camera
import math
from bearing import calculate_initial_compass_bearing

def gps_offset(gpsPosition):

    return (gpsPosition[0], gpsPosition[1]-0.2, gpsPosition[2]-0.2)

def difference(target, current):

    return (target[0] - current[0], target[1] - current[1], target[2] - current[2])
    
def bearing(displacement):

    initial_bearing = math.degrees(math.atan2(displacement[0],displacement[2]))
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing
    
def distance(displacement):

    return math.sqrt((displacement[0])**2 + (displacement[1])**2 + (displacement[2])**2)
    
def elevation(displacement, dist):

    return math.asin(displacement[1] / dist)
    
# def map_feature(

# create the Robot instance.
robot = Robot()

# set robot time step (multiple of world basicTimeStep)
TIME_STEP = 32

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
lidar = robot.getLidar('Velodyne HDL-32E')
lidar.enable(timestep)
gps = robot.getGPS('gps')
gps.enable(timestep)
imu = robot.getInertialUnit('imu') # need to center IMU, move GPS and change offset vals
imu.enable(timestep)
#camera = robot.getCamera('kinect')
#camera.enable(timestep)
wheels = []
wheelsNames = ['back left wheel', 'back right wheel',
             'front left wheel', 'front right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

SPEED = 1.0
TARGET_POSITION = (-1, 0.2, -1)

robot.step(timestep)
HOME_LOCATION = gps.getValues()
HOME_ROTATION = imu.getRollPitchYaw()

while robot.step(timestep) != -1:
    
    # Process sensor data here.
    currentGPSPos = gps.getValues()
    currentPos = gps_offset(currentGPSPos)
    displacement = difference(currentPos, TARGET_POSITION)
    currentBearing = (math.degrees(imu.getRollPitchYaw()[2]) + 360) % 360
    bearingToTarget = bearing(displacement)
    DistanceToTarget = distance(displacement)
    ElevationToTarget = elevation(displacement, DistanceToTarget)
    
    # print(currentGPSPos)
    # print(currentPos)
    # print(displacement)
    # print(currentBearing )
    print(currentBearing, "||||||||||||||", bearingToTarget)
    # print(DistanceToTarget)
    
    # print(currentBearing) 
    # print(bearingToTarget)
    # break
    
    #print(DistanceToTarget)
    
    if DistanceToTarget < 0.6:
        print("Start feature mapping")
        break
    elif abs(bearingToTarget - currentBearing) > 1:
        leftSpeed = 1.0
        rightSpeed = -1.0
    # elif abs(bearingToTarget - currentBearing) > 1:
        # leftSpeed = -1.0
        # rightSpeed = 1.0      
    else:
        leftSpeed = 1.0
        rightSpeed = 1.0
        
    # leftSpeed = -1.0
    # rightSpeed = -1.0   

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)


