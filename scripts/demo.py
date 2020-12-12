#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Turret():
	""" defining a class for turret"""
	def __init__(self):
		# this subscriber gets the image from gazebo camera
		sub = rospy.Subscriber('/camera/image',Image,self.image_callback)
		# the following lines initializes the publishers which control the angle of the servo motors.
		self.pub_x = rospy.Publisher('/laser_turret/servo_x_controller/command',Float64,queue_size=10)
		self.pub_y = rospy.Publisher('/laser_turret/servo_y_controller/command',Float64,queue_size=10)
		self.x = 0
		self.y = 0
		self.img = None
		self.bridge = CvBridge()

	def image_callback(self,data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# you may perform image processing like scaling, blurring etc here...

			# self.img = self.img[120:950, 300:1075]
			# height = 720
			# width = 720
			# dim = (height, width)
			# self.res = cv2.resize(self.img, dim, interpolation = cv2.INTER_LINEAR)					#resizing the image to 720 by 720 pixels
			# self.hsv = cv2.cvtColor(self.res, cv2.COLOR_BGR2HSV)
		except CvBridgeError as e:
 			print(e)

 	def position(self):
 		# you may use this function for calculating the position from the images
 		# self.x, self.y = some_algorithm_to_get_angle
 		# you may use the following for publishing the angles of the turret
 		self.pub_x.publish(self.x)
 		self.pub_y.publish(self.y)

if __name__ == '__main__':
	rospy.init_node('turret')
	rate = rospy.Rate(30)	# while loop running at 30 Hz
	T = Turret()
	while not rospy.is_shutdown:	
		T.position()
		rate.sleep()

		


