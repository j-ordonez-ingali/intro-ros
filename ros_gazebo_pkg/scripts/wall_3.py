#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
pub = rospy.Publisher('cmd_vel',Twist, queue_size=10)
linear_vel = 0.3
angular_vel = 0.2
diff = 0.03
sequence = 0
vel = Twist()
dist=5
def odom_callback(msg):
	global sequence
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) =euler_from_quaternion(orientation_list)

	a=len(msg.ranges)/2
	global dist
	dist=msg.ranges[a]

	
	
	if sequence == 0 :
		if dist>1:
			velocity.linear.x=0.2
		else:
			velocity.linear.x=0.0
			sequence=1
	if sequence==1:
		if msg.pose.pose.position.y<=2.0:
			vel.linear.x = linear_vel
		else:
			vel.linear.x = 0
			sequence=2
	if sequence==2:
		if yaw<=0.00:
			vel.angular.z = 0.0
			sequence=3
		else:
			vel.angular.z=angular_vel
	"""
	if sequence==3:
		if msg.pose.pose.position.x <=1.0:
			vel.linear.x=linear_vel
			
		else:
			vel.linear.x=0.0
			sequence=4
	if sequence == 4:
		if yaw<=-1.59+diff:
			vel.angular.z = 0.0
			sequence=5
		else:
			vel.angular.z=angular_vel
	if sequence==5:
		if msg.pose.pose.position.y>=1.0:
			vel.linear.x=linear_vel
			
		else:
			vel.linear.x=0.0
			sequence=6
	if sequence == 6:
		print (yaw)
		if yaw<=-3.1415+diff:
			vel.angular.z = 0.0
			sequence=7
		else:
			vel.angular.z=angular_vel
	if sequence==7:
		if msg.pose.pose.position.x>=0.0:
			vel.linear.x=linear_vel
			
		else:
			vel.linear.x=0.0
			sequence=8"""
	
			

	print (sequence)
	pub.publish(vel)
	#rospy.loginfo(rospy.get_caller_id() + "orientation %f", yaw) 
def subscriber():
	
	rospy.init_node('odom_subscriber', anonymous=True)

	rospy.Subscriber("odom", Odometry, odom_callback)

	rospy.spin()

if __name__ == '__main__':
	subscriber()
