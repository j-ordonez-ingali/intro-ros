#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

quat_msg=Quaternion()
class image_converter:

  def __init__(self):
    self.stop_pub = rospy.Publisher("stop",Float64)
    self.quat_pub = rospy.Publisher("filters",Quaternion)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
     
    lower_red=np.array([0,0,100])
    upper_red=np.array([50,50,255])
    lower_blue=np.array([100,0,0])
    upper_blue=np.array([255,50,50])
    lower_cyan=np.array([100,100,0])
    upper_cyan=np.array([255,255,50])
    lower_yellow=np.array([0,100,100])
    upper_yellow=np.array([50,255,255])
    lower_black=np.array([0,0,0])
    upper_black=np.array([50,50,50])
    red_mask=cv2.inRange(cv_image,lower_red,upper_red)
    blue_mask=cv2.inRange(cv_image,lower_blue,upper_blue)
    cyan_mask=cv2.inRange(cv_image,lower_cyan,upper_cyan)
    yellow_mask=cv2.inRange(cv_image,lower_yellow,upper_yellow)
    black_mask=cv2.inRange(cv_image,lower_black,upper_black)
    stop_mask=np.sum(red_mask)
    quat_msg.x=np.sum(blue_mask)
    quat_msg.y=np.sum(cyan_mask)
    quat_msg.z=np.sum(yellow_mask)
    quat_msg.w=np.sum(black_mask)
    cv2.imshow("Image window", cv_image)
    cv2.imshow("red_mask", red_mask)
    cv2.imshow("blue_mask", blue_mask)
    cv2.imshow("cyan_mask", cyan_mask)
    cv2.imshow("yellow_mask", yellow_mask)
    cv2.imshow("black_mask", black_mask)
    cv2.waitKey(3)

    try:
      self.stop_pub.publish(stop_mask)
      self.quat_pub.publish(quat_msg)
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
