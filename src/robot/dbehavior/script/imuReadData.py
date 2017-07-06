#!/usr/bin/env python

import rospy
from dmotion.msg import ActionCommand

def callback(data):
	rospy.loginfo(data)

def listener():
	rospy.init_node('imuListener', anonymous=True)
	rospy.Subscriber('/humanoid/ActionCommand', ActionCommand, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
