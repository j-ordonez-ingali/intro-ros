#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point

def DH_matrix(t,d,a,ap):
	A=np.array([[cos(t),-sin(t)*cos(ap),sin(t)*sin(ap),a*cos(t)],[sin(t),cos(t)*cos(ap),-cos(t)*sin(ap),a*sin(t)],[0,sin(ap),cos(ap),d],[0,0,0,1]])
	return A

def DH_subs(A,t1,t2,par):
	[m,n]=A.shape
	subs_A=np.zeros(A.shape)
	for i in range(0,m):
		for j in range(0,n):
			subs_A[i,j]=A[i,j].subs([(t1,par[0,0]),(t2,par[0,1])])
	return subs_A

A01=DH_matrix(0,0,0,0)
A12=DH_matrix(t1,0.4,0,np.pi/2)
A23=DH_matrix(t2,0,0.3,0)

A02=np.dot(A01,A12)
A03=np.dot(A02,A23)

def publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    pub_position = rospy.Publisher('position', Point, queue_size=10)

    rospy.init_node('urdf_joint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 1 message per second

    joint_movement = JointState()
    position=Point()

    header=Header()
    seq=0
    header.frame_id=''
    joints_names=['joint1','joint2','joint3']
    joint_movement.name=joints_names
    angle=[0.0,0.0,0.0]

    while not rospy.is_shutdown(): #similar ROS:OK / While ros master is working
	header.seq=seq
	header.stamp=rospy.Time.now()
	
	joint_movement.header=header
	angle[0]=angle[0]+0.1
	angle[1]=angle[1]+0.1
	joint_movement.position=angle
	

	params= np.array([[angle[0],angle[1]]])
	numeric_A= DH_subs(A03,t1,t2,params)
	position.x=numeric_A[0,3]
	position.y=numeric_A[1,3]
	position.z=numeric_A[2,3]
        
        pub.publish(joint_movement)
	pub.position(position)
        rate.sleep() #delay

	seq=seq+1

if __name__ == '__main__': #SImilar to int main
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
