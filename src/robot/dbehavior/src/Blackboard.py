from dmotion.msg import MotionInfo
from dvision.msg import VisionInfo
from dmotion.msg import ActionCommand
from dbehavior.msg import BehaviorInfo
from Parameters import getParam
from utils.actioncommand import crouch
import rospy

class Blackboard(object):
    def __init__(self):
        self.motionInfo = MotionInfo()
        self.visionInfo = VisionInfo()
        self.actionCmd = ActionCommand()
        self.behaviorInfo = BehaviorInfo()
        # update info
        rospy.Subscriber('/humanoid/MotionInfo', MotionInfo, self.updateMotionInfo)
        rospy.Subscriber('/humanoid/VisionInfo', VisionInfo, self.updateVisionInfo)
        self.cmdPub = rospy.Publisher('/humanoid/ActionCommand', ActionCommand, queue_size=1)
        self.behaviorInfoPub = rospy.Publisher('/humanoid/BehaviorInfo', BehaviorInfo, queue_size=1)
        self.parameters = getParam()

    def resetCmd(self):
        self.actionCmd = ActionCommand()
        self.actionCmd.bodyCmd = crouch()
        self.behaviorInfo = BehaviorInfo()
        self.behaviorInfo.save_image = False

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

