#!/usr/bin/env python
#test on the links

"""
workflow:
1. using Moveit! to control the arm
2. using /effort_finger_trajectory_controller 
(aka ros_control trajectory controller) to control the gripper
3. using ros_control position controller to control the finger tip
"""


import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list
import my_demotest
from gazebo_msgs.srv import DeleteModel, SpawnModel
import geometry_msgs.msg

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('testmove_kinova', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

tip_val = 0.0 #unit in radians

#publish the trajectory to Rviz
#if Rviz is not used, this command can be ignored
display_trajectory_publisher = rospy.Publisher('/move_group/display_planner_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


rospy.wait_for_service('/gazebo/unpause_physics')
unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
resp = unpause_gazebo()

rospy.wait_for_service("/gazebo/delete_model")
rospy.wait_for_service("/gazebo/spawn_sdf_model")
print(".................")
print("All services stand by.")
print(".................")
delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

with open("/home/li/catkin_ws/src/kinova-ros/kinova_gazebo/worlds/blocks.sdf", "r") as f:
  product_xml = f.read()

tip1_pub = rospy.Publisher('/j2n6s300/finger_tip_1_position_controller/command', Float64 ,queue_size=20)

tip2_pub = rospy.Publisher('/j2n6s300/finger_tip_2_position_controller/command', Float64 ,queue_size=20)

tip3_pub = rospy.Publisher('/j2n6s300/finger_tip_3_position_controller/command', Float64 ,queue_size=20)

def moveFingers (check,jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  print("here")
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    if check == "open":
    	
		point.positions.append(0.8)
		point.velocities.append(0)
		point.accelerations.append(0)
		point.effort.append(0)
    else:
    	point.positions.append(0.95)
    	point.velocities.append(-0.05)
        point.accelerations.append(-0.05)
        point.effort.append(1.7)
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  
  #keep the gripper closing during the whole movement
  while (count < 400):
    pub.publish(jointCmd)
    tip1_pub.publish(tip_val)
    tip2_pub.publish(tip_val)
    tip3_pub.publish(tip_val)
    count = count + 1
    rate.sleep()
  if count >= 500:
  	return True
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

moveFingers("open",[0.8,0.8,0.8],'j2n6s300',3)

#INITIALIZING VARIABLES FOR THE LOOP
i = 0   #COUNTER FOR EACH BLOCK
callFunction = []
callFunction = my_demotest.BlockLocate(i) #CAMERA CALL FUNCTION
tower_x = [-0.2, -0.26, -0.32,-0.2, -0.26, -0.32,-0.2, -0.26, -0.32] #X POSITION FOR TOWER
tower_y = [0.2, 0.2, 0.2, 0.26, 0.26, 0.26, 0.32, 0.32, 0.32] #Y POSITION FOR TOWER

block_x = [0.2, 0.2, 0.2, 0.3, 0.3, 0.3, 0.4, 0.4, 0.4]
block_y = [0.1, 0.2, 0.3,0.1, 0.2, 0.3, 0.1, 0.2, 0.3]
while i < 10:
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
	
	#close the gripper using effort-controller
    print("close gripper")
    delete_model("blue1")
    moveFingers("close",[-0.5,-0.5,-0.5],'j2n6s300',3)
    
    """
    eefpose = move_group.get_current_pose().pose
    new_eefpose = geometry_msgs.msg.Point()
    new_eefpose.x = eefpose.position.x
    new_eefpose.y = eefpose.position.y
    new_eefpose.z = eefpose.position.z
    orii = geometry_msgs.msg.Quaternion()
    orii.x = 0.0
    orii.y = 0.0
    orii.z = 0.0
    orii.w = 0.0
    item_pose = geometry_msgs.msg.Pose()
    item_pose.position = new_eefpose
    item_pose.orientation = orii
    """
    
    delay = rospy.Rate(3)
    delay.sleep()
    spawn_model("fuckblock", product_xml,"", pose_goal,"world")
    
    #RAISE ARM BACK UP IN SAME X-Y POSITION
    pose_goal.orientation.w = 0
    pose_goal.orientation.x = 1
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0

    pose_goal.position.x = posx
    pose_goal.position.y = posy
    pose_goal.position.z = 0.25

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=False)
    check1 = False
    count = 0
    prefix = 'j2n6s300'
    pub = rospy.Publisher('j2n6s300', JointTrajectory, queue_size=1)
    jointCmd = JointTrajectory()
    point = JointTrajectoryPoint()
    jointCmd.header.stamp = rospy.Time.now() + 	rospy.Duration.from_sec(0.0);
    point.time_from_start = rospy.Duration.from_sec(5.0)
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(1))
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(2))
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(3))
    point.positions.append([0.9,0.9,0.9])
    point.velocities.append(-0.5)
    point.accelerations.append(-0.5)
    point.effort.append(1.7)

    while check1 == False:
    	count += 1
    	jointCmd.points.append(point)
    	rate = rospy.Rate(100)
    	pub.publish(jointCmd)
    	rate.sleep()
    	if count > 500:
    		check1 = True
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
    moveFingers("open",[0.8,0.8,0.8],'j2n6s300',3)
    
    #INSTITUTING NEW VALUES FOR NEXT BLOCK
    
    i += 1
    callFunction = my_demotest.BlockLocate(i)

rospy.sleep(6)
