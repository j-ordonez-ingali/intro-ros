#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


def float64_callback(msg):

    rospy.loginfo(rospy.get_caller_id() + "received message %f", msg.data)
    
def subscriber():

    rospy.init_node('float64_subscriber', anonymous=True)

    rospy.Subscriber("number_float64", Float64, float64_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
