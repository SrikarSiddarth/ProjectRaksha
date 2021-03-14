#!/usr/bin/env python

import rospy
import numpy as np
import sys, select, os
import tty, termios
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

class Dataset(object):
	""" generates dataset """
	def __init__(self):
		self.y = [0,0,0]			# output coordinates
		self.x = [0,0,0]			# input coordinates

		sub = rospy.Subscriber('/gazebo/model_states/',ModelStates,self.model_states_cb)
		sub1 = rospy.Subscriber('depthSensor/points',Point,self.threeD_cb)

		# format - input_x, input_y, input_z, output_x, output_y, output_z
		self.outputArray = [[],[],[],[],[],[]]
		self.settings = termios.tcgetattr(sys.stdin)
		self.flag = 0

		

	def getKey(self):

		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
			key = sys.stdin.read(1)
		else:
			key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def model_states_cb(self,msg):
		self.y[0] = msg.pose[0].position.x
		self.y[1] = msg.pose[0].position.y
		self.y[2] = msg.pose[0].position.z
		# print(self.y)

	def threeD_cb(self, msg):
		self.x[0] = msg.x
		self.x[1] = msg.y
		self.x[2] = msg.z

	def saveArray(self):
		a = np.asarray(self.outputArray)
		np.save('dataset',a)
		print('array saved!!!')
		print(len(self.outputArray[0]))
		self.flag = 1

	def capture(self):
		key = self.getKey()
		# capture
		if key=='c':
			self.outputArray[0].append(self.x[0])
			self.outputArray[1].append(self.x[1])
			self.outputArray[2].append(self.x[2])
			self.outputArray[3].append(self.y[0])
			self.outputArray[4].append(self.y[1])
			self.outputArray[5].append(self.y[2])
			print('appended ')
			print(len(self.outputArray))
		# finish
		if key=='f':
			self.saveArray()
		# try:
		# 	key = self.getKey()
		# 	# capture
		# 	if key=='c':
		# 		self.outputArray[0].append(self.x[0])
		# 		self.outputArray[1].append(self.x[1])
		# 		self.outputArray[2].append(self.x[2])
		# 		self.outputArray[3].append(self.y[0])
		# 		self.outputArray[4].append(self.y[1])
		# 		self.outputArray[5].append(self.y[2])
		# 	# finish
		# 	if key=='f':
		# 		self.saveArray()
		# except:
		# 	print(e)



if __name__ == '__main__':
	rospy.init_node('dataset_generator')
	d = Dataset()
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		if d.flag:
			break
		d.capture()
		rate.sleep()

	