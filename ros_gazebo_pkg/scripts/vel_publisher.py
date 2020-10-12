#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def publisher():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
     #pub--> publisher object , number_float64 --> topic name, queue_size=10
    rospy.init_node('vel_publisher', anonymous=True)#initializing the node 
    rate = rospy.Rate(1) # 1hz  --> 1 message per second
    velocity =Twist()
    velocity.angular.z=3.14
    while not rospy.is_shutdown():
		velocity.linear.x=velocity.linear.x+0.01
		velocity.angular.z=velocity.angular.z-0.01
		pub.publish(velocity)
		rate.sleep()
if __name__ == '__main__':  ##Similar to int main(){}
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
