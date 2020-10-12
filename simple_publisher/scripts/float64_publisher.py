#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def publisher():
    pub = rospy.Publisher('number_float64', Float64, queue_size=10)
     #pub--> publisher object , number_float64 --> topic name, queue_size=10
    rospy.init_node('float64_publisher', anonymous=True)#initializing the node 

   	#float64_publisher-> node name, anonymous=True --> other nodes do not know who is sending messages
    rate = rospy.Rate(1) # 1hz  --> 1 message per second
    count =0.0

    while not rospy.is_shutdown():   ## similar to ROS:OK //while rosmaster is working
	count=count +2.56
        rospy.loginfo(count) ###similar to ROS_INFO
        pub.publish(count)
        rate.sleep()

if __name__ == '__main__':  ##Similar to int main(){}
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
