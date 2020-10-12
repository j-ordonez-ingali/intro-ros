#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

def publisher():
    pub = rospy.Publisher('positions', Point, queue_size=10)
     #pub--> publisher object , positions --> topic name, queue_size=10
    rospy.init_node('point_publisher', anonymous=True)#initializing the node 

   	#float64_publisher-> node name, anonymous=True --> other nodes do not know who is sending messages
    rate = rospy.Rate(1) # 1hz  --> 1 message per second
    position=Point()

    while not rospy.is_shutdown():   ## similar to ROS:OK //while rosmaster is working
	position.x = position.x +0.25 
	position.y = position.y +2.25 
	position.z = position.z -0.1 
        rospy.loginfo(position) ###similar to ROS_INFO
        pub.publish(position)
        rate.sleep()

if __name__ == '__main__':  ##Similar to int main(){}
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
