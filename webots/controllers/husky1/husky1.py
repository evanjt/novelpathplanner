""" Controller for ENGR90037 Husky navigation
    Authors: Evan Thomas evant1@student.unimelb.edu.au
             Josh Clough cloughj@student.unimelb.edu.au
     
    To be executed as a controller within a Webots simulator
"""

from controller import Robot, Lidar, InertialUnit

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Import Velodyne LiDAR 
velodyne = robot.getLidar("Velodyne HDL-32E")

# Import IMU
imu = robot.getInertialUnit("IMU")


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
print(imu.getRollPitchYaw())

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
