#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
import sys, select, os
import tty, termios

msg = """

Launch counter missile from your Laser Turret!
Press l key
Ctrl-C to quit

"""

e = """
Communications Failed
"""

def getKey():

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__ == '__main__':
	settings = termios.tcgetattr(sys.stdin)


	rospy.init_node('trig_turret_teleop')
	pub = rospy.Publisher('/trig',Empty,queue_size=10)
	try:
		print(msg)
		while(1):
			key=getKey()
			if key=='l':
				pub.publish()
			elif key=='\x03':
				break
			

	except:
		print(e)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
