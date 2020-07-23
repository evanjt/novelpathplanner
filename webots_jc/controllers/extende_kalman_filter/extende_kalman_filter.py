"""pioneer_py controller."""

# Code adapted from https://github.com/AtsushiSakai/PythonRobotics

"""
Extended kalman filter (EKF) localization sample
author: Atsushi Sakai (@Atsushi_twi)
"""
from controller import Robot, GPS, InertialUnit
import numpy as np
import math
import matplotlib.pyplot as plt

# create the Robot instance.
robot = Robot()

# set robot time step (multiple of world basicTimeStep) if different to world
TIME_STEP = 32

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get and enable devices/components
gps = robot.getGPS('gps')
gps.enable(timestep)
imu = robot.getInertialUnit('imu') # need to center IMU, move GPS and change offset vals
imu.enable(timestep)
wheels = []
wheelsNames = ['back left wheel', 'back right wheel',
             'front left wheel', 'front right wheel']
for i in range(4):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

SPEED = 1.0
TARGET_POSITION = (-1, 0.2, 1)

robot.step(timestep)
HOME_LOCATION = gps.getValues()
HOME_ROTATION = imu.getRollPitchYaw()

# Estimation parameter of EKF
Q = np.diag([1.0, 1.0])**2  # Observation x,y position covariance
R = np.diag([0.1, 0.1, np.deg2rad(1.0), 1.0])**2  # predict state covariance

#  Simulation parameter
Qsim = np.diag([0.5, 0.5])**2
Rsim = np.diag([1.0, np.deg2rad(30.0)])**2


def observation(xTrue, xd, u):

    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    zx = xTrue[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = xTrue[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.array([[zx, zy]])

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F.dot(x) + B.dot(u)

    return x


def observation_model(x):
    #  Observation Model
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H.dot(x)

    return z


def jacobF(x, u):
    """
    Jacobian of Motion Model
    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacobH(x):
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):

    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacobF(xPred, u)
    PPred = jF.dot(PEst).dot(jF.T) + R

    #  Update
    jH = jacobH(xPred)
    zPred = observation_model(xPred)
    y = z.T - zPred
    S = jH.dot(PPred).dot(jH.T) + Q
    K = PPred.dot(jH.T).dot(np.linalg.inv(S))
    xEst = xPred + K.dot(y)
    PEst = (np.eye(len(xEst)) - K.dot(jH)).dot(PPred)

    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
    R = np.array([[math.cos(angle), math.sin(angle)],
                  [-math.sin(angle), math.cos(angle)]])
    fx = R.dot(np.array([[x, y]]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    print(__file__ + " start!!")

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((1, 2))

    while robot.step(timestep) != -1:
        
        leftSpeed = 1.0
        rightSpeed = 1.0 
        
        v = 1.0  # [m/s]
        yawrate = 0.1  # [rad/s]
        u = np.array([[v, yawrate]]).T

        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.vstack((hz, z))

        if show_animation:
            plt.cla()
            plt.plot(hz[:, 0], hz[:, 1], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b")
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k")
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r")
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


if __name__ == '__main__':
    main()

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, InertialUnit #, Lidar, Camera
import math
import numpy as np

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
TARGET_POSITION = (-1, 0.2, 1)

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
    # print(bearingToTarget)
    # print(DistanceToTarget)
    
    if DistanceToTarget < 1:
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

    