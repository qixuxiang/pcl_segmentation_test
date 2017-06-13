#!/usr/bin/env python
"""
Created on: May 3, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

A service called by Vision
"""

import rospy
from geometry_msgs.msg import Twist, Vector3
from dmotion.msg import ActionCmd


def main():
    rospy.init_node('dbehavior_node', anonymous=True)
    pub = rospy.Publisher('/humanoid/ActionCommand', ActionCmd, queue_size=1)

    rate = rospy.Rate(1)
    rospy.loginfo('Start dbehavior node')

    while not rospy.is_shutdown():
        cmd = ActionCmd()

        cmd.gait_type = ActionCmd.CROUCH
        cmd.cmd_head_speed = Vector3(0, 1, 1)
        cmd.cmd_head = Vector3(0, 0, 0)

        # cmd.cmd_vel.linear = Vector3(random.randint(0, 6), 0, 0)
        # cmd.cmd_vel.angular = Vector3(0, 0, random.randint(0, 10))

        pub.publish(cmd)
        rospy.loginfo("Pub new cmd")

        rate.sleep()


if __name__ == '__main__':
    main()
