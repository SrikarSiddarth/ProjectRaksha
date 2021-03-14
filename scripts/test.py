#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Vector3
import time
if __name__ == '__main__':
	rospy.init_node('counter_attack_tester')
	pub = rospy.Publisher('/counterAttack',Twist,queue_size=15)
	pos = Vector3(0,0,10)
	force = Vector3(1400,0,850)
	msg = Twist(pos,force)
	t = time.time()
	while not rospy.is_shutdown():
		if time.time()-t>1:
			break
	pub.publish(msg)
	print('message published')
	
