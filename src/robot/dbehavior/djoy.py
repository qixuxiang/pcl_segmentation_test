#!/usr/bin/env python
"""
Created on: May 6, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

Microsoft Xbox 360 Wired Controller for Linux

Table of index number of /joy.buttons:

Index Button name on the actual controller [buttons]
0     A
1     B
2     X
3     Y
4     LB
5     RB
6     back
7     start
8     power
9     Button stick left
10    Button stick right

Table of index number of /joy.axis:

Index Axis name on the actual controller [axes]
0     Left/Right Axis stick left
1     Up/Down Axis stick left
2     LT
3     Left/Right Axis stick right
4     Up/Down Axis stick right
5     RT
6     cross key left/right
7     cross key up/down

Sample:

header: 
  seq: 3401
  stamp: 
    secs: 1494076054
    nsecs: 484258726
  frame_id: ''
axes: [0.00911218486726284, 0.026555921882390976, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

"""



import random
import rospy
from geometry_msgs.msg import Twist, Vector3
from dmotion.msg import ActionCmd
from sensor_msgs.msg import Joy

PUB = None


def callback(joy):
    yaw, pitch = joy.axes[3:5]
    yaw = yaw * 120
    pitch = pitch * 90

    y, x = joy.axes[0:2]
    x = x * 6
    y = y * 2

    LT, RT = joy.axes[2], joy.axes[5]
    LT = (1.0 - LT) / 2.0
    RT = (1.0 - RT) / 2.0
    t = (LT - RT) * 15.0

    cmd = ActionCmd()
    if abs(x) > 1 or abs(y) > 1 or abs(t) > 1:
        cmd.gait_type = ActionCmd.WENXI
        cmd.cmd_head = Vector3(0, pitch, yaw)
        cmd.cmd_head_speed = Vector3(0, 1, 1)

        cmd.cmd_vel.linear = Vector3(x, y, 0)
        cmd.cmd_vel.angular = Vector3(0, 0, t)
        rospy.loginfo("(x: %lf y: %lf t: %lf | pitch: %lf yaw: %lf" %(x, y, t, pitch, yaw))
    else:
        cmd.gait_type = ActionCmd.CROUCH

    PUB.publish(cmd)


def main():
    global PUB
    rospy.init_node('djoy_node', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    PUB = rospy.Publisher('/humanoid/ActionCommand', ActionCmd, queue_size=1)
    rospy.loginfo('Start joy node')
    rospy.spin()


if __name__ == '__main__':
    main()
