#!/usr/bin/env python
import rospy
import os
import Log
import pkgutil
from DecisionMaking.BehaviorTree.Task import Task
from Blackboard import getbb

def getSkill():
    bb = getbb()
    skill = bb.parameters.skill
    behaivorPackages = ["roles", "skills", "demo", "headskills"]
    foundSkill = False
    skillDir = os.getcwd()
    skillClass = None
    for package in behaivorPackages:
        modules = ['{}/{}'.format(skillDir, package)]
        allName = [name for _, name, _ in pkgutil.iter_modules(modules)]
        if skill not in allName:
            continue

        skillModule = __import__('{}.{}'.format(package, skill), fromlist=[skill])
        skillClass = getattr(skillModule, skill)
        foundSkill = True
        Log.info('\n[python] Module --- {} --- imported\n'.format(skill))

    if not foundSkill:
        raise ImportError('Can not find skill: {}'.format(skill))
    elif not issubclass(skillClass, Task):
        raise TypeError('Found skill is not a Task')

    return skillClass()


def main():
    rospy.init_node('behavior_node')
    skillInstance = getSkill()
    rate = rospy.Rate(50)
    bb = getbb()
    while not rospy.is_shutdown():
        bb.resetCmd()
        skillInstance.tick()
        bb.publish()
        rate.sleep()

if __name__ == '__main__':
    main()
