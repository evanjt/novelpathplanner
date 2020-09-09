#!/usr/bin/python3
import logging
import os
import threading

import research.navigation as nav
import research.constants as const
import research.logger as log

class RobotDevice():
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.wheels = []

    def addLidar(self, lidar3DID,
                 lidarFrontID = None,
                 lidarRearID = None):
        self.lidar = self.robot.getLidar(lidar3DID)
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        if lidarFrontID is not None:
            self.hokuyoFront = self.robot.getLidar(lidarFrontID)
            self.hokuyoFront.enable(self.timestep)
            self.hokuyoFront.enablePointCloud()

            # Set up parameters for Hokuyo 2D
            self.hkfWidth = self.hokuyoFront.getHorizontalResolution()
            self.hkfHalfWidth = self.hkfWidth / 2.0
            self.hkfMaxRange = self.hokuyoFront.getMaxRange()  # 30
            self.hkfRangeThreshold = self.hkfMaxRange / 20.0  # 1.5
            self.hkfValues = []
            self.hkfBraitenbergCoefficients = nav.getBraitenberg(self.robot,
                                                                 self.hkfWidth,
                                                                 self.hkfHalfWidth)
        if lidarRearID is not None:
            self.hokuyoRear = self.robot.getLidar(lidarRearID)
            self.hokuyoRear.enable(self.timestep)
            self.hokuyoRear.enablePointCloud()

    def addCamera(self, cameraRangeID):
        self.cameraRange = self.robot.getCamera(cameraRangeID)
        self.cameraRange.enable(self.timestep)
    
    def addGPS(self, gpsID):
        self.gps = self.robot.getGPS(gpsID)
        self.gps.enable(self.timestep)

    def addIMU(self, imuID):
        self.imu = self.robot.getInertialUnit(imuID)
        self.imu.enable(self.timestep)

    def addWheels(self, wheels):
        for wheel in wheels:
            self.wheels.append(self.robot.getMotor(wheel))

        [wheel.setPosition(float('inf')) for wheel in self.wheels]
        [wheel.setVelocity(0.0) for wheel in self.wheels]

    def startLogging(self):
        # Set up logger with formatting for file output
        logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-5s %(levelname)-5s %(message)s',
                    datefmt='%m-%d %H:%M:%S',
                    filename=os.path.join(const.OUTPUT_PATH, const.LOGFILENAME),
                    filemode='w')
        self.console = logging.StreamHandler()
        self.console.setLevel(logging.INFO)
        self.consoleformatter = logging.Formatter('%(asctime)s %(name)-5s: %(levelname)-5s %(message)s',
                                                  datefmt='%H:%M:%S')
        self.console.setFormatter(self.consoleformatter)
        logging.getLogger('').addHandler(self.console)

        # Start logging
        self.thread = threading.Thread(target = log.worker,
                                       args = (self.gps, self.imu,)) # Send in GPS/IMU objects
        self.thread.start()
