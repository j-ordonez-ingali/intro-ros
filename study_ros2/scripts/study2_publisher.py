#!/usr/bin/env python

import rospy
from sympy import *
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point


t1=Symbol('t1')
dp1=Symbol('dp1')
dp2=Symbol('dp2')
t2=Symbol('t2')

def DH_matrix(t,d,a,ap):
	A=np.array([[cos(t),-sin(t)*cos(ap),sin(t)*sin(ap),a*cos(t)],[sin(t),cos(t)*cos(ap),-cos(t)*sin(ap),a*sin(t)],[0,sin(ap),cos(ap),d],[0,0,0,1]])
	return A

def DH_subs(A,t1,dp1,dp2,t2,par):
	[m,n]=A.shape
	subs_A=np.zeros(A.shape)
	for i in range(0,m):
		for j in range(0,n):
			subs_A[i,j]=A[i,j].subs([(t1,par[0,0]),(dp1,par[0,1]),(dp2,par[0,2]),(t2,par[0,3])])
	return subs_A

A01=DH_matrix(0,0,0,0)
A12=DH_matrix(t1,0.4,0,np.pi/2)
A23=DH_matrix(np.pi/2,0,0,-np.pi/2)
A34=DH_matrix(0,-dp1-0.25,0,np.pi/2)
A45=DH_matrix(-np.pi/2,0,0,np.pi/2)
A56=DH_matrix(0,dp2-0.25,0,-np.pi/2)
A67=DH_matrix(t2,0,0.3,0)

A02=np.dot(A01,A12)
A03=np.dot(A02,A23)
A04=np.dot(A03,A34)
A05=np.dot(A04,A45)
A06=np.dot(A05,A56)
A07=np.dot(A06,A67)


print("--------------Px------------")
print(A07[0,3])
print("--------------Py------------")
print(A07[1,3])
print("--------------Pz------------")
print(A07[2,3])



def publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    pub_position = rospy.Publisher('position', Point, queue_size=10)

    rospy.init_node('study2_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1 message per second
    joint_movement = JointState()
    position=Point()

    header=Header()
    seq=0
    header.frame_id=''
    joints_names=['joint1','joint2','joint3','joint4','joint5']
    joint_movement.name=joints_names
    angle=[0.0,0.0,0.0,0.0,0.0]

    while not rospy.is_shutdown(): #similar ROS:OK / While ros master is working
	header.seq=seq
	header.stamp=rospy.Time.now()
	
	joint_movement.header=header
	angle[0]=angle[0]+0.01
	angle[1]=angle[1]+0.01
        angle[2]=angle[2]+0.01
	angle[3]=angle[3]+0.01
	angle[4]=0
	joint_movement.position=angle

	params=np.array([[angle[0],angle[1],angle[2],angle[3]]])
	numeric_A=DH_subs(A07,t1,dp1,dp2,t2,params)
	position.x=numeric_A[0,3]
	position.y=numeric_A[1,3]
	position.z=numeric_A[2,3]
	
        
        pub.publish(joint_movement)
        pub_position.publish(position)
        rate.sleep() #delay

	seq=seq+1

if __name__ == '__main__': #SImilar to int main
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
