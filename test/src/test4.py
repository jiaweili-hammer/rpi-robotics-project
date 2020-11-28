#!/usr/bin/env python
#test on cartesian paths

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
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

scale = 1
waypoints = []
wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1
wpose.position.y += scale *0.2
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.1
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan, wait=True)

