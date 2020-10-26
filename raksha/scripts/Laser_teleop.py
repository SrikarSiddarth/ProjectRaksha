#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

msg = """

Control your Laser Turret!

Rotating in x direction : 'a' & 'd'
Rotating in y direction : 'w' & 's'

Ctrl-C to quit

"""

e = """
Communications Failed
"""

def getKey():
	if os.name == 'nt':
		return msvcrt.getch()

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__ == '__main__':

	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)


	rospy.init_node('laser_turret_teleop')
	pub_x = rospy.Publisher('/laser_turret/servo_x_controller/command',Float64,queue_size=10)
	pub_y = rospy.Publisher('/laser_turret/servo_y_controller/command',Float64,queue_size=10)
	x = 0
	y = 0
	delta = 0.05
	try:
		print(msg)
		while(1):
			key=getKey()
			if key=='a':
				x += delta
			elif key=='d':
				x -= delta
			elif key=='w':
				y -= delta
			elif key=='s':
				y += delta
			elif key=='\x03':
				break

			if x>1.57:
				x=1.57
			if x<-1.57:
				x=-1.57
			if y>1.57:
				y=1.57
			if y<-1.57:
				y=-1.57

			pub_x.publish(x)
			pub_y.publish(y)

	except:
		print(e)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)