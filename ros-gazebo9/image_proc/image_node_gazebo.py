#!/usr/bin/env python

import sys
import rospy

import numpy as np
import cv2
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

image_bridge = CvBridge()
global_image = None


def image_callback(data):
	global global_image
	global image_bridge
	
	try:
		cv_image = image_bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)
	
	global_image = cv_image


if __name__ == '__main__':

	rospy.init_node("/img_node_for_sim", anonymous=True)
 	rospy.Subscriber("/webcam/image_raw", Image, image_callback)

	rate = rospy.Rate(2.0)

	camera = cv2.VideoCapture(0)

	if True:

		cv2.namedWindow("Gokmen UAV Cam", cv2.WINDOW_AUTOSIZE)

		while not rospy.is_shutdown():

			imageFrame = global_image

			if imageFrame is None:
				continue
                
                
            #do whatever you want like opencv bussiness


			cv2.imshow("Team Happy Cam", imageFrame)
			keyCode = cv2.waitKey(30) & 0xFF
		
            # Press ESC to stop the program
			if keyCode == 27:
				break
					
