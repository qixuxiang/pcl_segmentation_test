import rospy
import sys

class Parameters(object):
    """
    All rosparams.
    """
    def __init__(self):
        self.robotId = rospy.get_param('/ZJUDancer/RobotId')
        self.skill = rospy.get_param('/dbehavior/skill')
        self.enableLog = rospy.get_param('/dbehavior/enableLog')
        # TODO(MWX): add all parameters needed

if __name__ == '__main__':
    p = Parameters()
    print p.skill
    print p.enableLog
