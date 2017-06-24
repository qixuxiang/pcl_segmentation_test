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

        # Constants
        self.maxPitch = rospy.get_param('/dbehavior/constant/maxPitch')
        self.minPitch = rospy.get_param('/dbehavior/constant/minPitch')
        self.maxYaw = rospy.get_param('/dbehavior/constant/maxYaw')

_param = Parameters()

def getParam():
    global _param
    return _param

if __name__ == '__main__':
    p = Parameters()
    print p.skill
    print p.enableLog
