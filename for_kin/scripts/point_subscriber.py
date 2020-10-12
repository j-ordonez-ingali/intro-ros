#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

def point_callback(msg):

    rospy.loginfo(rospy.get_caller_id() + "received message in x: %f", msg.x)

    rospy.loginfo(rospy.get_caller_id() + "received message in y: %f", msg.y)

    rospy.loginfo(rospy.get_caller_id() + "received message in z: %f", msg.z)
    
def subscriber():

    rospy.init_node('point_subscriber', anonymous=True)

    rospy.Subscriber("positions", Point, point_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
