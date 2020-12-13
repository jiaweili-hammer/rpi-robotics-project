#!/usr/bin/env python
#test on the joints

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from sensor_msgs.msg import Image
#import opencv libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import boxes_finder as bf


rospy.loginfo("test log message to respy")


#print("creating CvBridge")
bridge = CvBridge()
#print("bridge created")
all_boxes = np.zeros((14, 2), dtype = "float32")

def image_callback(img_msg):
	print("image callback")
	cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

	boxes_remaining, red_boxes, green_boxes, blue_boxes = bf.find_boxes_in_rgb_image(cv_image)
	h = cv_image.shape[0]
	w = cv_image.shape[1]
	count_red_boxes = len(red_boxes)
	print('red box count: ', count_red_boxes)
	for n in range(count_red_boxes):
		print('red box: ', n, ' x: ', red_boxes[n][0], ' y: ', red_boxes[n][1])
	for n in range(len(green_boxes)):
		print('green box: ', n, ' x: ', green_boxes[n][0], ' y: ', green_boxes[n][1])
	for n in range(len(blue_boxes)):
		print('blue box: ', n, ' x: ', blue_boxes[n][0], ' y: ', blue_boxes[n][1])
	print("h x w", h, w)
	print("image callback complete")
	total_box_count = len(red_boxes) + len(green_boxes) + len(blue_boxes)
	print('total boxes: ', total_box_count)
	
	#while less than 9 boxes, add blue boxes
	# if more than 9 but less than 14, add green boxes
	# if on the 14th block, add red boxes
	for n in range(total_box_count):
		if(n < len(blue_boxes)):
			all_boxes[n][0] = blue_boxes[n][0]
			all_boxes[n][1] = blue_boxes[n][1]
		elif(n >= len(blue_boxes) and n < (len(blue_boxes) + len(green_boxes))):
			all_boxes[n][0] = green_boxes[n-len(blue_boxes)][0]
			all_boxes[n][1] = green_boxes[n-len(blue_boxes)][1]
		else:
			all_boxes[n][0] = red_boxes[n - len(blue_boxes) - len(green_boxes)][0]
			all_boxes[n][1] = red_boxes[n - len(blue_boxes) - len(green_boxes)][1]
		print('box: ', n+1, ' x: ', all_boxes[n][0], ' y: ', all_boxes[n][1])

#print("subscribing to image callback")
sub_image = rospy.Subscriber("/rrbot/camera1/image_raw", Image, image_callback)
#print("subscribing to image callback complete")

check = False

#returns location of associated block
#blue blocks are the lowest numbers
#green blocks are the middle numbers
#red blocks are the highest numbers
def BlockLocate(blockcount):
	x_Pos = 0
	y_Pos = 0
	#print('888888888888888888888888')
	print('block locate called')
	#print('888888888888888888888888')
	print("all_boxes length", len(all_boxes))
	for n in range(len(all_boxes)):
		print('box: ', n+1, ' x: ', all_boxes[n][0], ' y: ', all_boxes[n][1])
		
	#set x, y
	x_Pos = all_boxes[1]
	y_Pos = all_boxes[1]
	#set bool if last box
	if(blockcount == len(all_boxes)):
		check = True
	else:
		check = False
	return check, x_Pos, y_Pos
