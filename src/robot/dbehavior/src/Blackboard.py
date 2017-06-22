from dmotion.msg import MotionInfo
from dvision.msg import VisionInfo
from dmotion.msg import ActionCmd
from dbehavior.msg import BehaviorInfo
from Parameters import Parameters
import rospy

class Blackboard(object):
    def __init__(self):
        # sub
        self.motionInfo = MotionInfo()
        self.visionInfo = VisionInfo

        # pub
        self.actionCmd = ActionCmd()
        self.behaviorInfo = BehaviorInfo()

        # update info
        rospy.Subscriber('/humanoid/MotionInfo', MotionInfo, self.motionInfo)
        rospy.Subscriber('/humanoid/VisionInfo', VisionInfo, self.updateVisionInfo)
        self.cmdPub = rospy.Publisher('/humanoid/ActionCommand', ActionCmd, queue_size=1)
        self.behaviorInfoPub = rospy.Publisher('/humanoid/BehaviorInfo', BehaviorInfo, queue_size=1)
        self.parameters = Parameters()

    def updateMotionInfo(self, msg):
        self.motionInfo = msg

    def updateVisionInfo(self, msg):
        self.visionInfo = msg

    def publish(self):
        self.cmdPub.publish(self.actionCmd)
        self.behaviorInfoPub.publish(self.behaviorInfo)

"""
Blackboard singleton.
Global blackboard instance, everyone should use this same object, reading and writing this bb.
"""
_blackboard = Blackboard()

def getbb():
    """
    Get global blackboard instance.
    :return: Blackboard
    """
    global _blackboard
    return _blackboard

if __name__ == '__main__':
    rospy.init_node('testBlackboard')
    b = Blackboard()
    rospy.spin()

