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
joint_states.name=['j1','j2','j3','j4','j5']


px=0
py=0
pz=0
angle1=0
angle2=0
angle3=0
angle4=0
angle5=0
def substitution(J,t1,t2,t3,t4,ti):
    [m,n]=J.shape
    J_subs=np.zeros(J.shape)
    for i in range(0,m):
        for j in range(0,n):
            J_subs[i,j]=J[i,j].subs([(t1,ti[0,0]),(t2,ti[0,1]),(t3,ti[0,2]),(t4,ti[0,3])])
    return J_subs

t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
t4=Symbol('t4')
iterations=200;

def point_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard in x %f", data.x)
    rospy.loginfo(rospy.get_caller_id() + "I heard in y %f", data.y)
    rospy.loginfo(rospy.get_caller_id() + "I heard in z %f", data.z)
    pxyz= np.array([[ data.x, data.y, data.z]])
    alpha=0.1 #learning rate
    ti=np.array([[1.968, 0.936 ,0.667,2.689]]) # initial theta vector
    px= 0.4331*(-6.12323399573677e-17*sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) - 0.4331*(-6.12323399573677e-17*sin(t1)*cos(t2) - 1.0*sin(t2)*cos(t1))*cos(t3) - 2.64401243935914e-17*sin(t1)*sin(t2) + 1.24301650113456e-18*sin(t1)*cos(t2) + 0.1501*sin(t1) + 0.0203*sin(t2)*cos(t1) + 0.4318*cos(t1)*cos(t2)
    py= -0.4331*(-1.0*sin(t1)*sin(t2) + 6.12323399573677e-17*cos(t1)*cos(t2))*cos(t3) + 0.4331*(sin(t1)*cos(t2) + 6.12323399573677e-17*sin(t2)*cos(t1))*sin(t3) + 0.0203*sin(t1)*sin(t2) + 0.4318*sin(t1)*cos(t2) + 2.64401243935914e-17*sin(t2)*cos(t1) - 1.24301650113456e-18*cos(t1)*cos(t2) - 0.1501*cos(t1)
    pz= 0.4331*sin(t2)*sin(t3) + 0.4318*sin(t2) - 0.4331*cos(t2)*cos(t3) - 0.0203*cos(t2) + 0.6718

    J=np.array([[diff(px,t1),diff(px,t2),diff(px,t3),diff(px,t4)],[diff(py,t1),diff(py,t2),diff(py,t3),diff(py,t4)],[diff(pz,t1),diff(pz,t2),diff(pz,t3),diff(pz,t4)]])
    for i in range(0, iterations):
        e=pxyz-np.array([[px.subs([(t1,ti[0,0]),(t2,ti[0,1]),(t3,ti[0,2]),(t4,ti[0,3])]),py.subs([(t1,ti[0,0]),(t2,ti[0,1]),(t3,ti[0,2]),(t4,ti[0,3])]),pz.subs([(t1,ti[0,0]),(t2,ti[0,1]),(t3,ti[0,2]),(t4,ti[0,3])])]])
        J_subs=substitution(J,t1,t2,t3,t4,ti)
        dt=np.dot(np.linalg.pinv(J_subs),e.T)
        ti=ti+alpha*dt.T
    global angle1,angle2,angle3,angle4,angle5
    angle1=ti[0,0]
    angle2=ti[0,1]
    angle3=ti[0,2]
    angle4=ti[0,3]
    angle5=0
def listener():
    rospy.init_node('inverse_kin', anonymous=True)
    rospy.Subscriber("position", Point, point_callback)
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