from sympy import *
import numpy as np 
t1=Symbol('t1')
t2=Symbol('t2')
t3=Symbol('t3')
t4=Symbol('t4')


def DH_matrix(t,d,a,alph):
	T=np.array([[cos(t),-cos(alph)*sin(t),sin(alph)*sin(t),a*cos(t)],
		[sin(t),cos(alph)*cos(t),-sin(alph)*cos(t),a*sin(t)],
		[0,sin(alph),cos(alph),d],[0,0,0,1]])
	return T

def subs_dh(T,t1,t2,t3,t4,par):
	[m,n]=T.shape 
	T_subs=np.zeros(T.shape)
	for i in range(0,m):
		for j in range (0,n):
			T_subs[i,j]=T[i,j].subs([(t1,par[0,0]),(t2,par[0,1]),(t3,par[0,2]),(t4,par[0,3])])
	return T_subs

T01 = DH_matrix(t1,0.6718,0,np.pi/2)
T12 = DH_matrix(t2,0.1501,0.4318,0)
T23 = DH_matrix(t3,-0.0203,0,np.pi/2)
T34 = DH_matrix(t4,0.4331,0,np.pi/2)

T02=np.dot(T01,T12)
T03=np.dot(T02,T23)
T04=np.dot(T03,T34)


print("--------------Px------------")
print(T04[0,3])
print("--------------Py------------")
print(T04[1,3])
print("--------------Pz------------")
print(T04[2,3])

