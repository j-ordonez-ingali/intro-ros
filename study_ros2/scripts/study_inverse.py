#!/usr/bin/env python
import rospy
from sympy import *
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

joint_states=JointState()
header=Header()
header.frame_id=''
joint_states.name=['joint1','joint2','joint3','joint4','joint5']


px=0
py=0
pz=0
angle1=0
angle2=0
angle3=0
angle4=0
angle5=0
def substitution(J,t1,dp1,dp2,t2,ti):
    [m,n]=J.shape
    J_subs=np.zeros(J.shape)
    for i in range(0,m):
        for j in range(0,n):
            J_subs[i,j]=J[i,j].subs([(t1,ti[0,0]),(dp1,ti[0,1]),(dp2,ti[0,2]),(t2,ti[0,3])])
    return J_subs

t1=Symbol('t1')
dp1=Symbol('dp1')
dp2=Symbol('dp2')
t2=Symbol('t4')
iterations=200;

def target_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard in x %f", data.x)
    rospy.loginfo(rospy.get_caller_id() + "I heard in y %f", data.y)
    rospy.loginfo(rospy.get_caller_id() + "I heard in z %f", data.z)
    pxyz= np.array([[ data.x, data.y, data.z]])
    alpha=0.1 #learning rate
    ti=np.random.rand(1,4) # initial theta vector



    px= (-dp1 - 0.25)*(- 1.0*cos(t1)) + 0.3*(1.0*cos(t1))*cos(t2)


    py=(-dp1 - 0.25)*(-1.0*sin(t1)) + 0.3*(1.0*sin(t1))*cos(t2)


    pz= - 1.0*dp2 + 0.3*sin(t2) + 0.65


    J=np.array([[diff(px,t1),diff(px,dp1),diff(px,dp2),diff(px,t2)],[diff(py,t1),diff(py,dp1),diff(py,dp2),diff(py,t2)],[diff(pz,t1),diff(pz,dp1),diff(pz,dp2),diff(pz,t2)]])
    for i in range(0, iterations):
        e=pxyz-np.array([[px.subs([(t1,ti[0,0]),(dp1,ti[0,1]),(dp2,ti[0,2]),(t2,ti[0,3])]),py.subs([(t1,ti[0,0]),(dp1,ti[0,1]),(dp2,ti[0,2]),(t2,ti[0,3])]),pz.subs([(t1,ti[0,0]),(dp1,ti[0,1]),(dp2,ti[0,2]),(t2,ti[0,3])])]])
        J_subs=substitution(J,t1,dp1,dp2,t2,ti)
        dt=np.dot(np.linalg.pinv(J_subs),e.T)
        ti=ti+alpha*dt.T
    global angle1,angle2,angle3,angle4,angle5
    angle1=ti[0,0]
    angle2=ti[0,1]
    angle3=ti[0,2]
    angle4=ti[0,3]
    angle5=0
def listener():
    rospy.init_node('study_inverse', anonymous=True)
    rospy.Subscriber("position", Point, target_callback)
    joint_pub=rospy.Publisher('joint_states', JointState, queue_size=10)
    angle_pub=rospy.Publisher('angles', Quaternion, queue_size=10)
    rate = rospy.Rate(2)
    quaternion_msg=Quaternion()
    seq=0
    while not rospy.is_shutdown():
        header.seq=seq
        seq=seq+1
        header.stamp=rospy.get_rostime()
        joint_states.header=header
        joint_states.position=[angle1,angle2,angle3,angle4,angle5]
        joint_states.velocity=[]
        joint_states.effort=[]

        quaternion_msg.x=angle1
        quaternion_msg.y=angle2
        quaternion_msg.z=angle3
        quaternion_msg.w=angle4
        angle_pub.publish(quaternion_msg)

        rospy.loginfo("x:%f - y:%f - z:%f - w:%f ",angle1,angle2,angle3,angle4)
        joint_pub.publish(joint_states)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    listener()
