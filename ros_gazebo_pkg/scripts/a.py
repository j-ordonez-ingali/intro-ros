#!/usr/bin/env python
import sys
import copy 
import rospy 
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String


def move_group_python_interface():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('python_interface',anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("arm")
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
	print "============ Waiting for RVIZ..."
	rospy.sleep(10)
	print "============ Starting tutorial "
	print "============ Reference frame: %s" % group.get_planning_frame()
	print "============ End effector: %s" % group.get_end_effector_link()
	print "============ Robot Groups:"
	print robot.get_group_names()
	print "============ Printing robot state"
	print robot.get_current_state()
	print "============"
	print "============ Generating plan 1"
  	pose_target = geometry_msgs.msg.Pose()
  	cont=0
  	while (cont==0)
	  	#Vertice 1
	  	pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.2

	  	pose_target.position.y = -0.2
	  	pose_target.position.z = 0.3
		group.set_pose_target(pose_target)
		plan1 = group.plan()
		group.go()

		rospy.sleep(5)

	  	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan1)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 2
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.2

	  	pose_target.position.y = -0.2
	  	pose_target.position.z = 0.4
		group.set_pose_target(pose_target)
		plan2 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan2)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		
		#Vertice 3
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.2

	  	pose_target.position.y = -0.1
	  	pose_target.position.z = 0.4
		group.set_pose_target(pose_target)
		plan3 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan3)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 4
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.2

	  	pose_target.position.y = -0.1
	  	pose_target.position.z = 0.3
		group.set_pose_target(pose_target)
		plan3 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan3)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 5
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.3

	  	pose_target.position.y = -0.1
	  	pose_target.position.z = 0.3
		group.set_pose_target(pose_target)
		plan3 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan3)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 6
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.3

	  	pose_target.position.y = -0.1
	  	pose_target.position.z = 0.4
		group.set_pose_target(pose_target)
		plan3 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan3)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 7
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.3

	  	pose_target.position.y = -0.2
	  	pose_target.position.z = 0.4
		group.set_pose_target(pose_target)
		plan3 = group.plan()
		rospy.sleep(5)
		
		display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	  	display_trajectory.trajectory_start = robot.get_current_state()
	  	display_trajectory.trajectory.append(plan3)
	  	display_trajectory_publisher.publish(display_trajectory);

		group.go()
		rospy.sleep(5)

		#Vertice 8
		group.clear_pose_targets()

		pose_target.orientation.w = 1.0
	  	pose_target.position.x = 0.3

	  	pose_target.position.y = -0.2
	  	pose_target.position.z = 0.3
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
	
	moveit_commander.roscpp_shutdown()

	print "============ STOPPING"

if __name__=='__main__':
	try:
		move_group_python_interface()
  	except rospy.ROSInterruptException:
		pass
