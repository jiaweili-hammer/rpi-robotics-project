import cv2
import numpy as np

def find_boxes_in_bw_image(bw_img):
	# find the corners of boxes in the black and white input image
	#returns the number of boxes and an array of coordinate pairs
	img, contours, hierarchy = cv2.findContours(bw_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	box_count = len(contours)
	#RETR_TREE retrieves contours
	#CHAIN_APPROX_SIMPLE stores contours as just horizontal and vertical lines

	#print('countours found:', box_count)
	#for each box found, print out the coordinates
	for n in range(box_count):
		box_coordinates = contours[n]
		#print (' box ', n, ' coordinates:')
		#there will be 4 coordinate pairs because its a rectangle
		for c in range(4):
			corner = box_coordinates[c]
			x = corner[0][0]
			y = corner[0][1]
			#print ('corner ', c, 'x=', x, 'y=', y)

	return box_count, contours

def convert_corners_to_robot_world_coordinates(corners):
	#find mean value of x and y of corners to find center of box
	x_sum = 0
	y_sum = 0
	for c in range(4):
		x_sum = x_sum + corners[c][0][0]
		y_sum = y_sum + corners[c][0][1]
	x_mean = x_sum / 4
	y_mean = y_sum / 4
	
	#convert to robot space by scaling and translating the x/y values
	#the scaling and translating amounts will need to be adjusted based on world geometry and camera location etc.
	world_x = (101 - y_mean) /140.0
	world_y = (101 - x_mean) /140.0

	#print ('box center pixels: ', x_mean, y_mean, 'world:', world_x, world_y)
	return world_x, world_y

def convert_rgb_image_to_bw_image(rgb_image, red_min, red_max, green_min, green_max, blue_min, blue_max):
	#get width and height
	h = rgb_image.shape[0]
	w = rgb_image.shape[1]

	#create empty black and white image to hold output
	#each matrix is width x height containing single 8 bit values (gray scale but will only be pure white(255) or pure black (0))
	bw_image = np.zeros((h, w, 1), dtype = "uint8")

	#input image pixels are BGR format
	#print('converting pixels to binary black and white')
	#traverse each row of pixels
	for y in range(h):
		#traverse each pixel in the row
		for x in range(w):
			#get one color pixel. format is [Blue, Green, Red] 8 bit values
			px = rgb_image[y, x]
			#get blue, green, and red channel values for this pixel
			bc = px[0]
			gc = px[1]
			rc = px[2]

			#check for thresholds
			if ((bc >= blue_min) and (bc <= blue_max) and (gc >= green_min) and (gc <= green_max) and (rc >= red_min) and (rc <= red_max)):
				bw_image[y,x] = 255
			else:
				bw_image[y,x] = 0
	return bw_image

def find_boxes_in_rgb_image(rgb_image):
	#convert incoming rgb image into 3 black and white images,
	# each image will only contain the red cubes, green cubes, or the blue cubes

	red_box_img = convert_rgb_image_to_bw_image(rgb_image, 0, 128, 0, 128, 192, 255)
	green_box_img = convert_rgb_image_to_bw_image(rgb_image, 0, 128, 192, 255, 0, 128)
	blue_box_img = convert_rgb_image_to_bw_image(rgb_image, 192, 255, 0, 128, 0, 128)

	#save images to test the black and white images
	#print('saving images')
	#cv2.imwrite('blue-boxes.png', blue_box_img)
	#cv2.imwrite('green-boxes.png', green_box_img)
	#cv2.imwrite('red-boxes.png', red_box_img)
	#print('saved images')

	#find countours of red boxes
	#print('finding red boxes')
	red_box_count, red_box_corners = find_boxes_in_bw_image(red_box_img)
	red_boxes = np.zeros((red_box_count, 2), dtype = "float32")
	#print('red box count: ', red_box_count)
	for n in range(red_box_count):
		world_x, world_y = convert_corners_to_robot_world_coordinates(red_box_corners[n])
		red_boxes[n] = [world_x, world_y]
		#print('red x: ', world_x)
	#print('finding green boxes')
	green_box_count, green_box_corners = find_boxes_in_bw_image(green_box_img)
	green_boxes = np.zeros((green_box_count, 2), dtype = "float32")
	for n in range(green_box_count):
		world_x, world_y = convert_corners_to_robot_world_coordinates(green_box_corners[n])
		green_boxes[n] = [world_x, world_y]
		#print('green x: ', world_x)

	#print('finding blue boxes')
	blue_box_count, blue_box_corners = find_boxes_in_bw_image(blue_box_img)
	blue_boxes = np.zeros((blue_box_count, 2), dtype = "float32")
	for n in range(blue_box_count):
		world_x, world_y = convert_corners_to_robot_world_coordinates(blue_box_corners[n])
		blue_boxes[n] = [world_x, world_y]
		#print('blue x: ', world_x)

	sum_of_boxes = red_box_count + green_box_count + blue_box_count

	#print ('boxes remaining: ', sum_of_boxes)
	return sum_of_boxes, red_boxes, green_boxes, blue_boxes
		













