#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def callback(data):
    global pub
    rospy.loginfo(rospy.get_caller_id() + 'Received sensor value: %s', data.data)
    if data.data > 100:
        pub.publish(0)
    else:
        pub.publish(9)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('husky', anonymous=True)
    pub = rospy.Publisher('motor', Float64, queue_size=10)
    rospy.Subscriber("sensor", Float64, callback)

    #rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
