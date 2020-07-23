#!/usr/bin/env python

# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from controller import Robot


class MyController(Robot):
    def __init__(self):
        super(MyController, self).__init__()
        self.timeStep = 32  # set the control time step

        # get device tags
        self.distanceSensor = self.getDistanceSensor('my_distance_sensor')
        self.led = self.getLed('my_led')
        self.distanceSensor.enable(timeStep)  # enable sensors to read data from them

    def run(self):
        # main control loop: perform simulation steps of 32 milliseconds
        # and leave the loop when the simulation is over
        while self.step(self.timeStep) != -1:
            val = self.distanceSensor.getValue()  # Read and process sensor data
            self.led.set(1)                       # Send actuator commands


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


def talker():
    controller = MyController()
    controller.run()
    #pub = rospy.Publisher('husky', String, queue_size=10)
    #robot = Robot()
    #timeStep = int(robot.getBasicTimeStep())
    #left = robot.getMotor('motor.left')
    #right = robot.getMotor('motor.right')
    #sensor = robot.getDistanceSensor('prox.horizontal.2')  # front central proximity sensor
    sensor.enable(timeStep)


    rospy.Subscriber('motor', Float64, callback)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
