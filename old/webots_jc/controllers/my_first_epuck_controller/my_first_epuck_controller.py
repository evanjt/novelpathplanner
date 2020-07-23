"""my_first_epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# set max speed (radians)
MAX_SPEED = 6.28 

# set robot time step (multiple of world basicTimeStep)
TIME_STEP = 64

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
left_motor = robot.getMotor('left wheel motor')
right_motor = robot.getMotor('right wheel motor')
left_motor.setVelocity(0.5 * MAX_SPEED)
right_motor.setVelocity(0.5 * MAX_SPEED)
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # Process sensor data here.
    # detect obstacles
    right_obstacle = psValues[0] > 75.0 or psValues[1] > 75.0 or psValues[2] > 75.0
    left_obstacle = psValues[5] > 75.0 or psValues[6] > 75.0 or psValues[7] > 75.0

    # Enter here functions to send actuator commands, like:
    # move for 1000 radians
    left_motor.setPosition(1000)
    right_motor.setPosition(1000)
    # initialize motor speeds at 50% of MAX_SPEED.
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if left_obstacle:
        # turn right
        leftSpeed  += 0.5 * MAX_SPEED
        rightSpeed -= 0.5 * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  -= 0.5 * MAX_SPEED
        rightSpeed += 0.5 * MAX_SPEED
    # write actuators inputs
    left_motor.setVelocity(leftSpeed)
    right_motor.setVelocity(rightSpeed)
    pass

# Enter here exit cleanup code.
