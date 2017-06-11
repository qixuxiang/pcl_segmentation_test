#!/usr/bin/env python
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:10:23+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: main.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T13:12:29+08:00
# @Copyright: ZJUDancer

import rospy
from blackboard import BlackBoard
from std_msgs.msg import String


def mainloop():
    # init node
    rospy.init_node("dbehavior_node", anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("dbehavior node started")
    print "fuck"

    # init blackboard
    bb = BlackBoard()

    # start loop
    rospy.spin()


if __name__ == "__main__":
    mainloop()
