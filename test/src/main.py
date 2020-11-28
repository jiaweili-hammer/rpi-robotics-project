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
import my_demotest

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('testmove_kinova', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
hand_group = moveit_commander.MoveGroupCommander("gripper")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planner_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#Set and move the initial starting position of the arm
#Quaternion used here for roation of the end effector
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 0
pose_goal.orientation.x = 1
pose_goal.orientation.y = 0
pose_goal.orientation.z = 0
#Initial Position of the arm
pose_goal.position.x = 0.2
pose_goal.position.y = 0.0
pose_goal.position.z = 0.5
#Move the arm to the specified pos/orientation
move_group.set_pose_target(pose_goal)
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

joint_goal = [0, 0, 0]
joint_goal[0] = pi/4
joint_goal[1] = pi/4
joint_goal[2] = pi/4
plan = hand_group.plan(joint_goal)
hand_group.go(wait=True)

#INITIALIZING VARIABLES FOR THE LOOP
i = 0   #COUNTER FOR EACH BLOCK
callFunction = []
callFunction = my_demotest.BlockLocate(i) #CAMERA CALL FUNCTION
tower_x = [-0.2, -0.26, -0.32,-0.2, -0.26, -0.32,-0.2, -0.26, -0.32] #X POSITION FOR TOWER
tower_y = [0.2, 0.2, 0.2, 0.26, 0.26, 0.26, 0.32, 0.32, 0.32] #Y POSITION FOR TOWER

block_x = [0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4]
block_y = [0.1, 0.2, 0.3,0.1, 0.2, 0.3, 0.1, 0.2, 0.3]
while callFunction[0] == True:
    posx = block_x[i]
    posy = block_y[i]

    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = posx
    pose_goal.position.y = posy
    pose_goal.position.z = 0.2

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = posx
    pose_goal.position.y = posy
    pose_goal.position.z = 0.045

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    print("close gripper")
    joint_goal[0] = pi/3.35
    joint_goal[1] = pi/3.35
    joint_goal[2] = pi/3.35
    plan = hand_group.plan(joint_goal)
    hand_group.go(wait=True)

    #RAISE ARM BACK UP IN SAME X-Y POSITION
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = posx
    pose_goal.position.y = posy
    pose_goal.position.z = 0.25

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #MOVE ARM TO DESIRED X-Y POSITION FOR BLOCK TOWER
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = tower_x[i]
    pose_goal.position.y = tower_y[i]
    pose_goal.position.z = 0.25

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #LOWER ARM TO DESIRED Z POSITION FOR BLOCK TOWER
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = tower_x[i]
    pose_goal.position.y = tower_y[i]
    pose_goal.position.z = 0.1

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    #DROPPING THE BLOCK TO DESIRED LOCATION
    print("release block")
    joint_goal[0] = pi/4
    joint_goal[1] = pi/4
    joint_goal[2] = pi/4
    plan = hand_group.plan(joint_goal)
    hand_group.go(wait=True)
    
    #INSTITUTING NEW VALUES FOR NEXT BLOCK
    
    i += 1
    callFunction = my_demotest.BlockLocate(i)

rospy.sleep(6)
