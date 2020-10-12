from sympy import *
import numpy as np
#inverse kinematics for TR robot configuration
def substitution (J,t1,t2,ti):
	[m,n]=J.shape
	J_subs=np.zeros(J.shape)
	for i in range (0,n):
		for j in range (0,n):
			J_subs[i,j]=J[i,j].subs([(t1,ti[0,0]),(t2,ti[0,1])])
	return J_subs

t1=Symbol('t1')
t2=Symbol('t2')

##target point
#pxyz=np.array([[-0.24168,-0.142111,0.506741]]) #-2.61 0.3638 0.6478
#pxyz=np.array([[-0.00964912,-0.239824,0.220025]]) #1.53058  -2.4981  -0.91231
pxyz=np.array([[-0.00171976, -0.0448484,0.103376]])  #-1.6091  -1.406  -0.39185
	 

iterations= 200
alpha=0.1 #learning rate
ti=np.random.rand(1,2) ##remember that 2 depens on numbers of joints (in these case:t1 and t2)

px=0.3*cos(t1)*cos(t2)
py=0.3*sin(t1)*cos(t2)
pz=0.3*sin(t2) + 0.4 

##jacobian matrix

J=np.array([[diff(px,t1),diff(px,t2)],[diff(py,t1),diff(py,t2)],[diff(pz,t1),diff(pz,t2)]])

Ex=np.zeros(iterations)
Ey=np.zeros(iterations)
Ez=np.zeros(iterations)


for i in range (0,iterations):
	e=pxyz-np.array([[px.subs([(t1,ti[0,0]),(t2,ti[0,1])]),py.subs([(t1,ti[0,0]),(t2,ti[0,1])]),pz.subs([(t1,ti[0,0]),(t2,ti[0,1])])]])
	print e
	J_subs=substitution(J,t1,t2,ti)
	dt=np.dot(np.linalg.pinv(J_subs),e.T)
	ti=ti+alpha*dt.T
	Ex[i]=e[0,0]
	Ey[i]=e[0,1]
	Ez[i]=e[0,2]

print "----------Final result-----------"
print ti












