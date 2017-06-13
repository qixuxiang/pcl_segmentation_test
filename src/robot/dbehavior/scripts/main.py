#!/usr/bin/env python
"""Main script for dbehaviour."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:10:23+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: main.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T13:12:29+08:00
# @Copyright: ZJUDancer

import os
import rospy
import pkgutil
import misc.status.gglobal as gglobal
from misc.blackboard import BlackBoard
from misc.bt import Node, Root


skill_instance = None


def find_skill(skill):
    """Find skill."""
    skill_dir = os.path.join(os.getcwd(), "actions")
    behaviour_packages = ["roles", "game", "skills", "headskills"]
    # Load the module and the class we're going to use.
    found_skill = False
    skill_class = None
    for package in behaviour_packages:
        module = ["{}/{}".format(skill_dir, package)]
        all_name = [name for _, name, _ in pkgutil.iter_modules(module)]
        if skill not in all_name:
            continue

        skill_module = __import__("{}.{}".format(package, skill),
                                  fromlist=[skill])
        skill_class = getattr(skill_module, skill)
        found_skill = True
        rospy.loginfo("[python] Module --- {} --- imported".format(skill))
        break

    if not found_skill:
        raise ImportError("Could not find skill: {}".format(skill))

    return skill_class


def init_skill():
    """Init skill."""
    global skill_instance
    if skill_instance:
        return
    blackboard = gglobal.get_bb()
    skill = blackboard.behaviour.skill
    skill_class = find_skill(skill)
    if issubclass(skill_class, Node):
        skill_instance = Root()
        skill_instance.add_child(skill_class)
    else:
        raise TypeError('skill found is not subclass of Node')


def mainloop():
    """Main loop for dbehaviour."""
    # init node
    rospy.init_node("dbehavior_node", anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("dbehavior node started")

    # init blackboard
    bb = BlackBoard()
    gglobal.update_bb(bb)

    # start loop
    # rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        global skill_instance
        # init behaviour request
        gglobal.init_req()
        # init skill instance
        init_skill()
        # tick skill
        skill_instance.tick()
        # publish behaviour request
        req = gglobal.get_req()
        req.publish()
        # sleep
        rate.sleep()


if __name__ == "__main__":
    mainloop()
