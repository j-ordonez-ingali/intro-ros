#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    gray_image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    try:
    	circles = cv2.HoughCircles(gray_image,cv.CV_HOUGH_GRADIENT,1,30,param1=50,param2=30,minRadius=10,maxRadius=80)
        circles = np.uint16(np.around(circles))
    except cv2.error as e:
    	None
    for i in circles[0,:]:
		cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
    '''
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    edges=cv2.Canny(cv_image,100,200)
    '''
    
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
