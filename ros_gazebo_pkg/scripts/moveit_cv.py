#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import copy 
import rospy 
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
import cv2.cv as cv
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError







def move_group_python_interface():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('python_interface',anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("arm")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	print("============ Waiting for RVIZ...")
	rospy.sleep(10)
	print("============ Starting tutorial ")
	print("============ Reference frame: %s" % group.get_planning_frame())
	print("============ End effector: %s" % group.get_end_effector_link())
	print("============ Robot Groups:")
	print(robot.get_group_names())
	print("============ Printing robot state")
	print(robot.get_current_state())
	print("============")

	cv_image=cv2.imread('/home/ubuntu/imt_ws/src/ros_gazebo_pkg/scripts/circulos.jpg');
	(rows,cols,channels) = cv_image.shape
	print(rows)
	print(cols)
	print(channels)
    	gray_image=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
	try:
    		circles = cv2.HoughCircles(gray_image,cv.CV_HOUGH_GRADIENT,1,30,param1=50,param2=30,minRadius=10,maxRadius=80)
        	circles = np.uint16(np.around(circles))
	except cv2.error as e:
    		None
	
	print("============ Generating plan 1")
  	pose_target = geometry_msgs.msg.Pose()
	
	
	for i in circles[0,:]:
		print(0.2+float(i[0])/3200)
		print(0.1+float(i[1])/4000)
		print(pose_target.position.y)
	  	pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.2+float(i[0])/3200
	  	pose_target.position.y = 0.1
	  	pose_target.position.z = 0.1+float(i[1])/4000
		group.set_pose_target(pose_target)
		plan1 = group.plan()
#	group.go()
		
		rospy.sleep(5)
	  	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  		display_trajectory.trajectory_start = robot.get_current_state()
  		display_trajectory.trajectory.append(plan1)
  		display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)
		group.clear_pose_targets()
	''''
	group.clear_pose_targets()

	pose_target.orientation.w = 1.0
  	pose_target.position.x = 0.3

  	pose_target.position.y = 0.25
  	pose_target.position.z = 0.35
	group.set_pose_target(pose_target)
	plan2 = group.plan()
	rospy.sleep(5)
	
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  	display_trajectory.trajectory_start = robot.get_current_state()
  	display_trajectory.trajectory.append(plan2)
  	display_trajectory_publisher.publish(display_trajectory);

	group.go()
	rospy.sleep(5)

	

	group.clear_pose_targets()

	pose_target.orientation.w = 1.0
  	pose_target.position.x = 0.3

  	pose_target.position.y = 0.25
  	pose_target.position.z = 0.35
	group.set_pose_target(pose_target)
	plan3 = group.plan()
	rospy.sleep(5)
	
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  	display_trajectory.trajectory_start = robot.get_current_state()
  	display_trajectory.trajectory.append(plan3)
  	display_trajectory_publisher.publish(display_trajectory);

	group.go()
	rospy.sleep(5)


	group_variable_values = group.get_current_joint_values()
	print "============ Joint values: ", group_variable_values
	group.set_joint_value_target(group_variable_values)
	print "============ Joint values: ", group_variable_values
  	plan2 = group.plan()

	rospy.sleep(5)
	'''
	
	moveit_commander.roscpp_shutdown()

	print("============ STOPPING")

if __name__=='__main__':
	try:
		move_group_python_interface()
  	except rospy.ROSInterruptException:
		pass
