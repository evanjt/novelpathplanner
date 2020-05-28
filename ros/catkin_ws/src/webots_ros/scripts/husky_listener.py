#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64


def callback(data):
    global pub
    rospy.loginfo(rospy.get_caller_id() + 'Received sensor value: %s', data.data)
    if data.data > 100:
        pub.publish(0)
    else:
        pub.publish(9)


rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('motor', Float64, queue_size=10)
rospy.Subscriber("sensor", Float64, callback)
rospy.spin()
