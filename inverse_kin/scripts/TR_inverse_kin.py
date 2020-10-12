#!/usr/bin/env python
from sympy import *
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point


pub = rospy.Publisher('joint_states', JointState, queue_size=10)
joint_movement = JointState()


    

joints_names=['joint1','joint2','joint3']
joint_movement.name=joints_names
angle=[0.0,0.0,0.0]


t1=Symbol('t1')
t2=Symbol('t2')

def substitution(J,t1,t2,ti):
	[m,n]=J.shape
	J_subs=np.zeros(J.shape)
	for i in range (0,m):
		for j in range(0,n):
			J_subs[i,j]=J[i,j].subs([(t1,ti[0,0]),(t2,ti[0,1])])
	return J_subs

iterations=200
alpha=0.1 #learning rate
ti=np.random.rand(1,2)

px=0.3*cos(t1)*cos(t2)
py=0.3*sin(t1)*cos(t2)
pz=0.3*sin(t2) + 0.4


J=np.array([[diff(px,t1),diff(px,t2)],[diff(py,t1),diff(py,t2)],[diff(pz,t1),diff(pz,t2)]])



def target_callback(msg):
    header=Header()
    seq=0
    header.frame_id=''
    ti=np.random.rand(1,2)
    angle=[0.0,0.0,0.0]
    pxyz=np.array([[msg.x,msg.y,msg.z]])

    for i in range(0,iterations):
	e=pxyz-np.array([[px.subs([(t1,ti[0,0]),(t2,ti[0,1])]),py.subs([(t1,ti[0,0]),(t2,ti[0,1])]),pz.subs([(t1,ti[0,0]),(t2,ti[0,1])])]])
	
	J_subs=substitution(J,t1,t2,ti)
	dt=np.dot(np.linalg.pinv(J_subs),e.T)
	ti=ti+alpha*dt.T
    print ti
    header.seq=seq
    header.stamp=rospy.Time.now()
    joint_movement.header=header
    angle[0]=ti[0,0]
    angle[1]=ti[0,1]
    joint_movement.position=angle
    pub.publish(joint_movement)

    seq=seq+1
     
def subscriber():

    rospy.init_node('TR_inverse_kin', anonymous=True)

    rospy.Subscriber("target", Point, target_callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()
