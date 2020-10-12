#!/usr/bin/env python
import numpy as np
from sympy import *


t=Symbol ("t")

angle = 0.56

rt_m = Array([[cos(t),-sin(t)],[sin(t),cos(t)]])

print (rt_m)


rm=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])

print ("determinant")
print (np.linalg.det(rm))

print ("Transpose")
print (rm.T)

print ("Inverse")
print (np.linalg.inv(rm))

print ("rotation matrix inverse is equal to transpose")

print ("Evaluating vectors")

angle=np.pi/4
vector1=np.array([[1,1]])
rotated_vector1=np.dot(vector1,rm);
print("original vector")
print (vector1)
print ("comparing magnitudes")
print ("original: ",np.sqrt(np.sum(vector1**2)))
print ("original: ",np.sqrt(np.sum(rotated_vector1**2)))


print ("-----------double rotation-------------------")
angle=np.pi/4
angle2=np.pi/4
rm1=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
rm2=np.array([[np.cos(angle2),-np.sin(angle2)],[np.sin(angle2),np.cos(angle2)]])

print ("original vector")
vector1=np.array([[1,1]])
print (vector1)
double_rotated=np.dot(np.dot(vector1,rm1),rm2)
print ("result")
print (double_rotated)


print ("----------3D rotation---------------")
vector3D=np.array([[0],[0],[1]])
angle=np.pi/2
rmx=np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
rmy=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
rmz=np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
print ("Rotated matrix")
rotated3D=np.dot(rmy,vector3D)
print (rotated3D)

print ("double 3D rotation")
vector3D=np.array([[0],[1],[1]])
print("original: ", vector3D)
angle=-np.pi/4
rmx=np.array([[1,0,0],[0,np.cos(angle),-np.sin(angle)],[0,np.sin(angle),np.cos(angle)]])
rotated3D2=np.dot(rmz,rotated3D1)
print ("Rotated matrix")
print (rotated3D2)
