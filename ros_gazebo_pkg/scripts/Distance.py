#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

dist=10
def scan_callback(msg):
    a=len(msg.ranges)/2
    rospy.loginfo(rospy.get_caller_id() + "Distance at the center %f", msg.ranges[a])
    global dist
    dist=msg.ranges[a]
    
    
def sub_pub():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('Distance', anonymous=True)
    
    rospy.Subscriber("scan", LaserScan, scan_callback)
    global dist
    rate = rospy.Rate(1) # 1hz  --> 1 message per second
    velocity =Twist()
    
    while not rospy.is_shutdown():

	if dist>1:
		velocity.linear.x=0.2
	else:
		velocity.linear.x=-0.2
		
	pub.publish(velocity)
	rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

"""
def publisher():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
     #pub--> publisher object , number_float64 --> topic name, queue_size=10
    rospy.init_node('vel_publisher', anonymous=True)#initializing the node 
    global dist
    rate = rospy.Rate(1) # 1hz  --> 1 message per second
    velocity =Twist()
    velocity.angular.z=3.14
    while not rospy.is_shutdown():

	if dist>2:
		velocity.linear.x=0.1
	else:
		velocity.linear.x=0
		
	pub.publish(velocity)
	rate.sleep()

"""

if __name__ == '__main__':  ##Similar to int main(){}
    try:
        sub_pub()
    except rospy.ROSInterruptException:
        pass
