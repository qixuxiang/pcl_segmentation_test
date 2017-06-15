#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3
from dmotion.msg import ActionCmd

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Motion!
---------------------------
Moving around:
        w    
   a    s    d

x : turn around
z : turn left; c : turn right

q : left kick; e : right kick

m : golie
j : set up from back down
k : set up from front down

r/anything else : crounch

,/. : increase/decrease only linear speed by 10%
</> : increase/decrease only angular speed by 10%

h : print help page

CTRL-C to quit
"""

moveBindings = {
		'w':(3, 0, 0),
		'a':(0, 1, 0),
		's':(-1, 0, 0),
		'd':(0, -1, 0),
		'x':(0, 0, 1),
		'z':(1, 0, 1),
		'c':(1, 0, -1),
		}

speedBindings={
		',':(1.1,1),
		'.':(.9,1),
		'<':(1,1.1),
		'>':(1,.9),
		}

kickBindings = {
		'q': ActionCmd.LEFT_KICK,
		'e': ActionCmd.RIGHT_KICK,
		}

goalieBinding = 'm'
standupBackBinding = 'j'
standupFrontBinding = 'k'

helpBinding = 'h'

settings = termios.tcgetattr(sys.stdin)

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def main():
	speed = 1.5
	turn = 2
	pitch = 40
	yaw = 0

	cmd = ActionCmd()
	cmd.cmd_head = Vector3(0, pitch, yaw)
	cmd.cmd_head_speed = Vector3(0, 1, 1)

	rospy.init_node('demomain_node', anonymous=True)
	pub  = rospy.Publisher('humanoid/ActionCommand', ActionCmd, queue_size = 1)
	rospy.loginfo('Start DemoMain')

	print msg

	while not rospy.is_shutdown():
		key = getKey()
		if key in moveBindings.keys():
			x = moveBindings[key][0] * speed
			y = moveBindings[key][1] * speed
			th = moveBindings[key][2] * turn
			if abs(x) > 1 or abs(y) > 1 or abs(th) > 1:
				cmd.gait_type = ActionCmd.WENXI
				cmd.cmd_vel.linear = Vector3(x, y, 0)
				cmd.cmd_vel.angular = Vector3(0, 0, th)
			else:
				cmd.gait_type = ActionCmd.CROUCH
		elif key in kickBindings.keys():
			cmd.gait_type = ActionCmd.KICK
			cmd.kick_side = kickBindings[key]

		elif key in speedBindings.keys():
			speed = speed * speedBindings[key][0]
			turn = turn * speedBindings[key][1]
			print vels(speed,turn)		
		elif key == helpBinding:
			print msg		
		elif key == goalieBinding:
			cmd.gait_type = ActionCmd.GOALIE
		elif key == standupBackBinding:
			cmd.gait_type = ActionCmd.SETUPBACK
		elif key == standupFrontBinding:
			cmd.gait_type = ActionCmd.SETUPFRONT

		else:
			cmd.gait_type = ActionCmd.CROUCH
			if (key == '\x03'):
				break

		pub.publish(cmd)

if __name__ == '__main__':
	main()
