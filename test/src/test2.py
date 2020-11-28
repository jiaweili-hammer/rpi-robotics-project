#!/usr/bin/env python
#test on the links

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('testmove_kinova', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planner_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 0.8
pose_goal.orientation.y = 0
pose_goal.orientation.z = 0
pose_goal.orientation.w = 1
pose_goal.position.x = 0.4
pose_goal.position.y = 0.2
pose_goal.position.z = 0.6

move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

