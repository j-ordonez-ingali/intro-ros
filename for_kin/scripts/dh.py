from sympy import *
import numpy as np

t1=Symbol('t1')
t2=Symbol('t2')
L1=Symbol('L1')
L2=Symbol('L2')


def DH_matrix(t,d,a,ap):
	A=np.array([[cos(t),-sin(t)*cos(ap),sin(t)*sin(ap),a*cos(t)],[sin(t),cos(t)*cos(ap),-cos(t)*sin(ap),a*sin(t)],[0,sin(ap),cos(ap),d],[0,0,0,1]])
	return A

def DH_subs(A,t1,t2,L1,L2,par):
	[m,n]=A.shape
	subs_A=np.zeros(A.shape)
	for i in range(0,m):
		for j in range(0,n):
			subs_A[i,j]=A[i,j].subs([(t1,par[0,0]),(t2,par[0,1]),(L1,par[0,2]),(L2,par[0,3])])
	return subs_A


A01= DH_matrix(0,0,0,np.pi/2)
A12= DH_matrix(t1,0,L1,0)
A23= DH_matrix(t2,0,L2,0)

#Multipliying MAtrices

A02=np.dot(A01,A12)
A03=np.dot(A02,A23)


print ("------FINAL MATRIX---------")
print(A03)
print("--------POSITION----------")
print("-------PX--------")
print(A03[0,3])
print("-------PY--------")
print(A03[1,3])
print("-------PZ--------")
print(A03[2,3])


print ("------------Evaluating some cases------------")
print ("case A")
params =np.array([[0,0,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case B")
params =np.array([[np.pi/2,0,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case C")
params =np.array([[0,np.pi/2,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case D")
params =np.array([[np.pi/2,np.pi/2,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case E")
params =np.array([[np.pi/4,0,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case f")
params =np.array([[np.pi/4,np.pi/2,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])


print ("case g")
params =np.array([[-np.pi/2,-np.pi/2,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])

print ("case h")
params =np.array([[np.pi,-np.pi/2,5,3]])
numeric_A=DH_subs(A03,t1,t2,L1,L2,params)
print ("Px=",numeric_A[0,3],"Py=",numeric_A[1,3],"Pz=",numeric_A[2,3])




