#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import Point,Twist,Vector3
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2 as cv

class Turret():
	""" defining a class for turret"""
	def __init__(self):
		print('eem')
		

		self.x = 1	
		self.y = -1
		self.img = None
		self.hsv = None
		self.realPos= [None,None,None]
		self.pos = [0,0,0]									# position of the target from gazebo
		self.hDistance = 0
		self.counterAttackFlag = 0
		self.turretHeight = 5.2
		self.origin = (0,0,self.turretHeight)
		self.forceMagnitude = 2500							# newtons
		self.counterAttackPos = Vector3(self.origin[0],self.origin[1],self.origin[2])
		self.force = Vector3()
		self.counterAttackMsg = Twist()
		self.bridge = CvBridge()
		self.p = Point()

		# this subscriber gets the image from gazebo camera
		sub = rospy.Subscriber('/camera/color/image_raw',Image,self.image_callback)
		sub1 = rospy.Subscriber('depthSensor/points',Point,self.threeD_cb)
		sub2 = rospy.Subscriber('/gazebo/model_states/',ModelStates,self.model_states_cb)
		sub_trig = rospy.Subscriber('/trig',Empty,self.trigger_missile_cb)
		# the following lines initializes the publishers which control the angle of the servo motors.
		self.pub_x = rospy.Publisher('/laser_turret/servo_x_controller/command',Float64,queue_size=10)
		self.pub_y = rospy.Publisher('/laser_turret/servo_y_controller/command',Float64,queue_size=10)
		self.pubxy = rospy.Publisher('xyPoint',Point,queue_size=10)
		self.pub_counter_attack = rospy.Publisher('/counterAttack',Twist,queue_size=15)
		
		# self.fgbg = cv.bgsegm.createBackgroundSubtractorMOG()

	def trigger_missile_cb(self,msg):
		self.counterAttackFlag = 1

	def model_states_cb(self,msg):
		self.pos[0] = msg.pose[0].position.x
		self.pos[1] = msg.pose[0].position.y
		self.pos[2] = msg.pose[0].position.z
		# print(self.pos)

	def threeD_cb(self, msg):
		self.realPos[0] = msg.z - 9.50973892212			
		self.realPos[1] = -1*msg.x + 0.0359052307904
		self.realPos[2] = 15.0809831619 - msg.y
		# difference gazebo positions and calculated positions from the sensor
		# print([abs(self.realPos[i] - self.pos[i]) for i in range(3)])
		self.hDistance = np.sqrt((self.realPos[0]-0)**2+(self.realPos[1]-0)**2)

	def image_callback(self,data):
		try:
			# print('om')
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# you may perform image processing like scaling, blurring etc here...

			# lower_red = np.array([165,155,95])
			# upper_red = np.array([180,255,255])
			self.hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
			# mask = cv.inRange(hsv, lower_red, upper_red)
			# res = cv.bitwise_and(self.img,self.img, mask= mask)

			lower_red_1 = np.array([0, 100, 100])
			upper_red_1 = np.array([10, 255, 255])
			mask_red_1 = cv.inRange(self.hsv, lower_red_1, upper_red_1)
			lower_red_2 = np.array([160, 100, 100])
			upper_red_2 = np.array([179, 255, 255])
			mask_red_2 = cv.inRange(self.hsv, lower_red_2, upper_red_2)
			mask_red = cv.addWeighted(mask_red_1, 1, mask_red_2, 1, 0)

			# gray = cv.cvtColor(mask_red, cv.COLOR_BGR2GRAY)
			# blur = cv.GaussianBlur(gray,(5,5),0)
			# _ , thresh = cv.threshold(blur,20,255,cv.THRESH_BINARY)
			# dilated = cv.dilate(thresh,None,iterations = 3)
			_,contours, _ = cv.findContours(mask_red, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
			# (x,y,w,h) = (0,0,0,0)
			cArea = 0
			index = 0
			for i in range(len(contours)):
				# print(cv.contourArea(count),len(contours))
				c = cv.contourArea(contours[i])
				if c>cArea:
					cArea = c
					index = i
			(x,y,w,h) = cv.boundingRect(contours[index])
			x2 = x + int(w/2)
			y2 = y + int(h/2)
			cv.circle(self.img,(x2,y2),4,(0,255,0),-1)
			text = "x: " + str(x2) + ", y: " + str(y2)
			cv.putText(self.img, text, (x2 - 10, y2 - 10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			
				
			# fgmask = self.fgbg.apply(mask_red)
			# x2 = x + int(w/2)
			# y2 = y + int(h/2)
			# cv.circle(self.img,(x2,y2),4,(0,255,0),-1)
			# text = "x: " + str(x2) + ", y: " + str(y2)
			# cv.putText(self.img, text, (x2 - 10, y2 - 10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			self.img = cv.resize(self.img, (540,480), interpolation = cv.INTER_LINEAR)
			cv.imshow('self.img',self.img)
			# cv.imshow('FG',fgmask)
			cv.waitKey(33)
			self.p.x = x2
			self.p.y = y2
			self.pubxy.publish(self.p)
			# print(x2, y2)
		except CvBridgeError as e:
 			print(e)

 	def get_cosines(self):
 		dist = np.sqrt((self.realPos[0]-self.origin[0])**2 + (self.realPos[1]-self.origin[1])**2 + (self.realPos[2] - self.origin[2])**2)
 		return (self.realPos[0]-self.origin[0])/dist,(self.realPos[1]-self.origin[1])/dist,(self.realPos[2]-self.origin[2])/dist
 		

 	def position(self):
 		# you may use this function for calculating the position from the images
 		# self.x, self.y = some_algorithm_to_get_angle
 		# you may use the following for publishing the angles of the turret
 		if self.realPos[0] is not None:
	 		self.x = np.arctan((self.realPos[1]-0)/(self.realPos[0]-0))
	 		self.y = -1*np.arctan((self.realPos[2] - self.turretHeight)/self.hDistance)
	 		cx,cy,cz = self.get_cosines()

	 		self.pub_x.publish(self.x)
	 		self.pub_y.publish(self.y)
	 		# print(self.x,self.y)
	 		if self.counterAttackFlag:
	 			self.force = Vector3(self.forceMagnitude*cx,self.forceMagnitude*cy,self.forceMagnitude*cz)
	 			print('Force : '+str(self.force))
	 			self.counterAttackMsg = Twist(self.counterAttackPos,self.force)
	 			self.pub_counter_attack.publish(self.counterAttackMsg)
	 			self.counterAttackFlag = 0


if __name__ == '__main__':
	rospy.init_node('turret')
	rate = rospy.Rate(30)	# while loop running at 30 Hz
	T = Turret()
	while not rospy.is_shutdown():	
		T.position()
		rate.sleep()

		


