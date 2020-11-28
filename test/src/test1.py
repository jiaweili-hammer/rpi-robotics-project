#!/usr/bin/env python
#test on the joints

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

joint_state_topic = ['joint_states:=/j2n6s300/joint_states']


moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('testmove_kinova', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "gripper"
move_group = moveit_commander.MoveGroupCommander(group_name)
#print move_group.get_current_pose()
print("...............................")
print move_group.get_current_joint_values()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planner_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = pi/4
joint_goal[1] = pi/4
joint_goal[2] = pi/4

plan = move_group.plan(joint_goal)
move_group.go(wait=True)
rospy.sleep(6)

move_group.stop()

joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0

plan = move_group.plan(joint_goal)
move_group.go(wait=True)
rospy.sleep(3)

move_group.stop()
