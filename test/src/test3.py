#!/usr/bin/env python
#test on grasping objects

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
rospy.init_node('grasp_kinova', anonymous=True)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planner_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_name = "arm"
arm_group = moveit_commander.MoveGroupCommander(arm_name)
hand_group = moveit_commander.MoveGroupCommander("gripper")
#arm_group.set_planner_id("RRTConnectkConfigDefault")
arm_group.set_planning_time(5)
arm_group.set_goal_tolerance(0.1)
print("===ready to start===")
# fisr movement, move close to the object
pose_goal = geometry_msgs.msg.Pose()
"""
pose_goal.orientation.w = 0
pose_goal.orientation.x = -0.6
pose_goal.orientation.y = 0.78
pose_goal.orientation.z = 0
"""

pose_goal.orientation.w = 1
pose_goal.orientation.x = 0
pose_goal.orientation.y = 0
pose_goal.orientation.z = 0

pose_goal.position.x = 0.16
pose_goal.position.y = 0.46
pose_goal.position.z = 0.2

arm_group.set_pose_target(pose_goal)
plan1 = arm_group.go(wait=True)
arm_group.stop()
print('===stage1 finished===')


# open the gripper
joint_goal = hand_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0
joint_goal[2] = 0
hand_group.go(joint_goal, wait=True)
#rospy.sleep(5)
hand_group.stop()
#hand_group.clear_pose_targets()
print('===stage2 finished===')

pose_goal.position.z = 0.15
arm_group.set_pose_target(pose_goal)
plan3 = arm_group.go()
arm_group.stop()
print('===stage3 finished===')

joint_goal[0] = pi/4
joint_goal[1] = pi/4
joint_goal[2] = pi/4
hand_group.go(joint_goal,wait=True)
hand_group.stop()
print('===stage4 finished===')

pose_goal.position.x = -0.5
pose_goal.position.y = -0.5
pose_goal.position.z = 0.5
arm_group.set_pose_target(pose_goal)
plan4 = arm_group.go(wait=True)
arm_group.stop()

values = arm_group.get_current_pose()
print(values)

