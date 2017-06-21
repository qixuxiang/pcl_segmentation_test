#!/usr/bin/env python
import os
import rospy
import pkgutil
import misc.status.gglobal as gglobal
from misc.blackboard import BlackBoard
from misc.bt import Node, Root


skill_instance = None


def find_skill(skill):
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

        skill_module = __import__("{}.{}.{}".format("actions", package, skill),
                                  fromlist=[skill])
        skill_class = getattr(skill_module, skill)
        found_skill = True
        rospy.loginfo("[python] Module --- {} --- imported".format(skill))
        break

    if not found_skill:
        raise ImportError("Could not find skill: {}".format(skill))

    return skill_class


def init_skill():
    global skill_instance
    if skill_instance:
        return
    blackboard = gglobal.get_bb()
    skill = blackboard.params.skill
    skill_class = find_skill(skill)
    if issubclass(skill_class, Node):
        skill_instance = Root()
        skill_instance.add_child(skill_class)
    else:
        raise TypeError('skill found is not subclass of Node')


def mainloop():
    rospy.init_node("dbehavior_node", anonymous=True, log_level=rospy.INFO)
    rospy.loginfo("dbehavior node started")

    bb = BlackBoard()
    gglobal.update_bb(bb)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        global skill_instance
        gglobal.update_bb(bb)
        gglobal.init_req()
        init_skill()
        skill_instance.tick()
        req = gglobal.get_req()
        req.publish()
        rate.sleep()


if __name__ == "__main__":
    mainloop()
