"""husky1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

velodyne = robot.getLidar("Velodyne HDL-32E")

# Turn the Velodyne LiDAR on
velodyne.enable(timestep)
velodyne.enablePointCloud()
print("Point cloud enabled: {}".format(velodyne.isPointCloudEnabled()))
print("Numer of layers: {}".format(velodyne.getNumberOfLayers()))
print("Horizontal resolution: {}".format(velodyne.getHorizontalResolution()))
print("Frequency: {}".format(velodyne.getFrequency()))
# print(velodyne.getNumberOfPoints())
# print(velodyne.getPointCloud())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # pointcloud = velodyne.getPointCloud()    
    # print(velodyne.getPointCloud()[0].x,
    # velodyne.getPointCloud()[0].y,
    # velodyne.getPointCloud()[0].z,
    # velodyne.getPointCloud()[0].layer_id,
    # velodyne.getPointCloud()[0].time)
    # print(len(pointcloud))
    # for point in pointcloud:
        # print(point.layer_id)
    # print(velodyne.getNumberOfPoints())
    # controller.
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
