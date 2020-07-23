"""pioneer_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Lidar, GPS, Camera, PositionSensor
from math import sqrt

def euclidean_dist(x2, y2, z2):
    """
    Helper to calc 3D euclidean distance. Assumes first coordinate (0,0,0)
    which is ideal for relative LiDAR positions.
    """
    
    #Assumes position of feture to map
    x1, y1, z1 = (-1, 0.25, -1)
    return sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

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
#camera = robot.getCamera('kinect')
#camera.enable(timestep)
wheels = []
sensors = []
wheelsNames = ['back left wheel', 'back right wheel',
             'front left wheel', 'front right wheel']
sensorNames = ['back left wheel sensor', 'back right wheel sensor',
             'front left wheel sensor', 'front right wheel sensor']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    sensors.append(robot.getPositionSensor(sensorNames[i]))
    sensors[i].enable(timestep)
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
#print(gps.getSamplingPeriod())

last = 0.0

SPEED = 1.0

# Main loop:
# - perform 100 simulation steps
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
    X = gps.getValues()[0]
    Y = gps.getValues()[1]
    Z = gps.getValues()[2]
    
    #print("%.6f, %.6f, %.6f" %(X, Y, Z))
    
    current = euclidean_dist(X, Y, Z)
    print(current)
    print(last)
    
    if current < 0.2:
        print("within mapping dist")
    elif last < current:
        leftSpeed = -1.0
        rightSpeed = 1.0
    else:
        leftSpeed = 1.0
        rightSpeed = 1.0   
    
    last = current

    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
