#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from mavros_msgs.msg import State, HomePosition
from sensor_msgs.msg import NavSatFix
from from nav_msgs.msg import Odometry
from mavros_msgs.srv import *
from geometry_msgs.msg import Twist, PoseStamped
import time


# ################################################################################################################################################
# -------------------------------------------------------------- MOVE ON AXIS ---------------------------------------------------
# ################################################################################################################################################

from decimal import *
from math import cos, sin, asin, sqrt, radians
import math

# degree to radian
def deg2rad(deg):
	return deg * (math.pi/180)

# Haversine formula, use distance of two point that include lat-lon in radian
# It calculates distance of two coordinate points in kmeter

def haversine():
  	global lat
  	global lon
  	global prev_lat
  	global prev_lon

   	# enlem-boylam verilerini radyan cinsinde tutma
   	r_lat, r_lon, r_prev_lat, r_prev_lon = deg2rad(lat), deg2rad(lon), deg2rad(prev_lat), deg2rad(prev_lon)

   	# enlem-boylam verilerinin farki (radyan) 
	diff_lat = abs(r_lat - r_prev_lat) 
	diff_lon = abs(r_lon - r_prev_lon) 

    # haversine formula
	a = sin(diff_lat/2)**2 + cos(r_prev_lat) * cos(r_lat) * sin(diff_lon/2)**2
	c = 2 * asin(sqrt(a)) 

	R = 6371 #radius of earth (km)

	#print(Decimal(c * R * 1000))

	return Decimal(c * R * 1000) # distance in meter




def moveX(dist, speed): #(in meter (float), lineer speed(float))
	global lat
	global lon
	global prev_lat
	global prev_lon

	msg.linear.x = speed # lineer speed on x-axis

	while not rospy.is_shutdown():
		dist_covered = haversine() 
		if dist_covered > dist:
			break

		velocity_pub.publish(msg)

		rate.sleep()

	print("Enlem: {:.7f} ,Boylam: {:.7f}\nAlinan Yol: {:.7f}".format(lat, lon, dist_covered))

	# Dur komutu
	msg.linear.x = 0.
	msg.linear.y = 0.
	msg.linear.z = 0.
	for i in range(100):
		velocity_pub.publish(msg)

	rospy.sleep(1.)

	# after movement, re-assaign initializing position info
	prev_lat = lat
	prev_lon = lon



dist_diff = 0.0

def moveY(dist, speed, round): #(in meter (float), lineer speed(float))
	global lat
	global lon
	global prev_lat
	global prev_lon
	global okColor

	global dist_diff

	msg.linear.y = speed # lineer speed on y-axis

	while not rospy.is_shutdown():
		dist_covered = haversine() 
		if round == 1 and speed < 0:
			print("Havuz bulundu ? {}".format(okColor))
			if okColor == "Red":
				print("Kirmizi boylam: {}, Kirmizi enlem: {} ".format(red_lon,red_lat))
			if okColor == "Blue":
				print("Mavi boylam: {}, Mavi enlem: {} ".format(blue_lon,blue_lat))

		if round == 2 and speed < 0:
			if abs(red_lon - lon) < 0.1 and abs(red_lat - lat) < 0.1:
				dist_diff = dist - dist_covered
				break
			if abs(blue_lon - lon) < 0.1 and abs(blue_lat - lat) < 0.1:
				dist_diff = dist - dist_covered
				break
		if dist_covered > dist:
			break

		velocity_pub.publish(msg)
		rate.sleep()

	print("Enlem: {:.7f} ,Boylam: {:.7f}\nAlinan Yol: {:.7f}".format(lat, lon, dist_covered))

	# Dur komutu
	msg.linear.x = 0.
	msg.linear.y = 0.
	msg.linear.z = 0.
	
	for i in range(100):
		velocity_pub.publish(msg)

	rospy.sleep(1.)

	# after movement, re-assaign initializing position info
	prev_lat = lat
	prev_lon = lon



def moveZ(dist, speed): #(in meter (float), lineer speed(float))
	global alt
	global prev_alt

	msg.linear.z = speed # lineer speed on z-axis

	while not rospy.is_shutdown(): 
		if speed > 0 and alt > dist + prev_alt:
			break
		if speed < 0 and alt < prev_alt - dist:
			break

		velocity_pub.publish(msg)

		rate.sleep()

	print("Ilk iritifa: {:.7f} ,Anlik irtifa: {:.7f}\n, Fark: {:.7f}".format(prev_alt, alt, alt - prev_alt))

	# Dur komutu
	msg.linear.x = 0.
	msg.linear.y = 0.
	msg.linear.z = 0.
	for i in range(100):
		velocity_pub.publish(msg)


	rospy.sleep(1.)

	# after movement, re-assaign initializing position info
	prev_alt = alt

def turnLeft(angle_deg, clockwise = False):

	speed_r = 0.405

	angle_r = deg2rad(angle_deg)

	if clockwise:
		msg.angular.z = -abs(speed_r)
		msg.angular.x = 0
		msg.angular.y = 0
	else:
		msg.angular.z = abs(speed_r)
		msg.angular.x = 0
		msg.angular.y = 0

	
	t0 = rospy.Time.now().secs

	current_angle = 0

	while(current_angle < angle_r):

		velocity_pub.publish(msg)

		t1 = rospy.Time.now().secs

		current_angle = speed_r * (t1 - t0)

		rate.sleep()

	msg.linear.x = 0.
	msg.linear.z = 0.
	msg.linear.y = 0.
	msg.angular.z = 0.

	for i in range(50):
		velocity_pub.publish(msg)

	rospy.sleep(1.)