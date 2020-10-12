#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def scan_callback(msg):
    a=len(msg.ranges)/2
    rospy.loginfo(rospy.get_caller_id() + "Distance at the center %f", msg.ranges[a])
    
def subscriber():

    rospy.init_node('scan_subscriber', anonymous=True)

    rospy.Subscriber("scan", LaserScan, scan_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
