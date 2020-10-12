#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2

import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
	def __init__(self):
		self.image_pub=rospy.Publisher("image2",Image)
		self.brazo_pub=rospy.Publisher("seven_dof_arm/joint2_position_controller/command",Float64,queue_size=20)
		self.bridge=CvBridge()
		self.image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
	
	def callback(self,data):
		try:
			cv_image=self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)
		#ret, im_bin=cv2.threshold(cv_image,127,255,cv2.THRESH_BINARY)

		lower =np.array([0,0,50], dtype=np.uint8)
		upper =np.array([80,80,255], dtype=np.uint8)

		mask=cv2.inRange(cv_image,lower,upper)

		
		#angle=0.0f
		cv2.imshow("mask",mask)
		#cv2.imshow("left",imleft)
		#cv2.imshow("right",imrigth)
		imleft=mask[:,0:325]
		imrigth=mask[:,325:650]
		suma_right=np.sum(imrigth)
		suma_left=np.sum(imleft)
		if (suma_left<suma_right) :
			rospy.loginfo("Izquierda")
			angle=0.1+((90%100)*3.14159)/200
			self.brazo_pub.publish(angle)
		else:
			rospy.loginfo("Derecha")
			angle=0.1+((0%100)*3.14159)/200
			self.brazo_pub.publish(angle)
		
		cv2.waitKey(3)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
		except CvBridgeError as e:
			print(e)

def main(args):
	ic=image_converter()
	rospy.init_node("image_brige",anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()

if __name__=='__main__':
	main(sys.argv)
