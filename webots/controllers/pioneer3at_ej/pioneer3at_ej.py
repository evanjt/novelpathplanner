"""pioneer_py controller."""

from controller import Robot, Lidar, GPS, InertialUnit, Camera, RangeFinder
import math

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

# create the Robot instance.
robot = Robot()

# set robot time step (multiple of world basicTimeStep)
TIME_STEP = 80

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get and enable robot devices
lidar = robot.getLidar('Velodyne HDL-32E')
lidar.enable(timestep)
gps = robot.getGPS('gps')
gps.enable(timestep)
imu = robot.getInertialUnit('imu')
imu.enable(timestep)
camera = robot.getCamera('kinect color')
camera.enable(timestep)
kinectRange = robot.getRangeFinder('kinect range')
kinectRange.enable(timestep)
kinect_width = kinectRange.getWidth()
kinect_height = kinectRange.getHeight()
half_width = kinect_width / 2
view_height = kinect_height / 2 + 10
min_range = kinectRange.getMinRange()
max_range = kinectRange.getMaxRange()
range_threshold = 1.5
inv_max_range_times_width = 1.0 / (max_range * kinect_width)
wheels = []
wheelNames = ['back left wheel', 'back right wheel',
             'front left wheel', 'front right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

SPEED = 1.0
TARGET_POSITION = (1, 0.2, 1)

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

    # get range-finder values
    # kinect_values = kinectRange.getRangeImageArray()

    # for i in range(round(kinect_width)):
    # record near obstacle sensed on the left side
        # value = kinectRange.rangeImageGetDepth(kinect_values, round(kinect_width), i, round(view_height))
        # if value < range_threshold:  # far obstacles are ignored
            # left_obstacle += value
            # print(left_obstacle)
        # record near obstacle sensed on the right side
        # value = kinectRange.rangeImageGetDepth(kinect_values, round(kinect_width), round(kinect_width) - i, round(view_height))
        # if value < range_threshold:
            # right_obstacle += value

    # obstacle = left_obstacle + right_obstacle

    # print(obstacle)
    # print(currentGPSPos)
    # print(currentPos)
    # print(displacement)
    # print(currentBearing )
    # print(bearingToTarget)
    # print(DistanceToTarget)

    if DistanceToTarget < 1.5:
        print("prepare to map features")
        newBearing = currentBearing + 90
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
                while robot.step(timestep) != -1: # change to forloop/detect return to starting pos
                    leftSpeed = 1.0
                    rightSpeed = 1.0 * 0.3333

                    wheels[0].setVelocity(leftSpeed)
                    wheels[1].setVelocity(rightSpeed)
                    wheels[2].setVelocity(leftSpeed)
                    wheels[3].setVelocity(rightSpeed)
                break

                #return home code
        break
    elif abs(bearingToTarget - currentBearing) > 1:
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:
        leftSpeed = 1.0
        rightSpeed = 1.0

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)

