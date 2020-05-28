#!/usr/bin/env python3

# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.data)
    velocity = data.data


def talker():
    pub = rospy.Publisher('husky', String, queue_size=10)
    rospy.Subscriber('motor', Float64, callback)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
