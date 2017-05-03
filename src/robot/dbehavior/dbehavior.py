#!/usr/bin/env python
"""
Created on: May 3, 2017
    Author: Wenxing Mei <mwx36mwx@gmail.com>

A service called by Vision
"""

import rospy
from geometry_msgs.msg import Twist, Vector3


def main():
    rospy.init_node('dbehavior_node', anonymous=True)
    body_pub = rospy.Publisher('/humanoid/dbehavior/cmd_vel', Twist, queue_size=1)
    head_pub = rospy.Publisher('/humanoid/dbehavior/cmd_head', Vector3, queue_size=1)

    rate = rospy.Rate(10)
    rospy.loginfo('Start dbehavior node')
    while not rospy.is_shutdown():
        body = Twist()
        body.linear = [3, 0, 0]
        body.angular = [0, 0, 3]
        body_pub.publish(body)

        head = Vector3()
        head.x = 0  # roll
        head.y = 15  # pitch
        head.z = 15  # yaw
        head_pub.publish(head)

        rate.sleep()


if __name__ == '__main__':
    main()
