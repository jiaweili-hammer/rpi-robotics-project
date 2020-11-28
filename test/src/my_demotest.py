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

x_Pos = 0
y_Pos = 0
def BlockLocate(blockcount):
    if blockcount <= 5:
        check = True
    else:
        check = False
    x_Pos = 0.2
    y_Pos = blockcount/10 + 0.1


    return [check, x_Pos, y_Pos]
