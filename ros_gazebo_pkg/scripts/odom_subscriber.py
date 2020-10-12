#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

linear_vel = 0.1
angular_vel = 0.08
velocity =Twist()
sequence=0

def odom_callback(msg):
    
    global sequence
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    
    if sequence==0:
        if yaw<=3.1416/2:
            velocity.angular.z=-angular_vel
        else:
            velocity.angular.z=0.0
            sequence=1
	if sequence==1:
		if msg.pose.pose.position.y<=1.0:
            velocity.linear.x=linear_vel
        else:
            velocity.linear.x=0.0
            sequence=2
    if sequence==2:
		if yaw<=0.03:
            velocity.angular.z=0.0
		else:
            velocity.linear.x=angular_vel
            sequence=3
    if sequence==3:
		if msg.pose.pose.position.x<=1.0:
            velocity.linear.x=linear_vel
		else:
            velocity.linear.x=0.0
            sequence=4

	print (sequence)

    pub.publish(velocity)
    #rospy.loginfo(rospy.get_caller_id() + "Orientation in z %f", yaw)

   

    
def subscriber():

    rospy.init_node('odom_subscriber', anonymous=True)

    rospy.Subscriber("odom", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()
