#!/usr/bin/env python
import rospy
from Blackboard import getbb


def initSkills():
    bb = getbb()
    skillName = bb.parameters.skill
    # TODO(MWX): find skill
    skillInstance = None
    return skillInstance

def main():
    rospy.init_node('behavior_node')
    skillInstance = initSkills()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        skillInstance.tick()
        bb = getbb()
        bb.publish()


if __name__ == '__main__':
    main()
