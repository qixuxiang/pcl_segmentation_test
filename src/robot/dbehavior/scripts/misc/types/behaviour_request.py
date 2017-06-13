"""class BehaviourRequest."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-13T10:43:41+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: behaviour_request.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-13T10:43:52+08:00
# @Copyright: ZJUDancer

import rospy
from geometry_msgs.msg import Vector3
from dmotion.msg import ActionCmd
from ..types.constant import UNKNOWN
from ..types.action_command import crouch, head


class BehaviourRequest(object):
    """BehaviourRequest."""

    def __init__(self):
        """Init."""
        super(BehaviourRequest, self).__init__()
        self.update_gait_vec = False
        self.actions = ActionCmd()
        crouch(self.actions)
        head(self.actions, 0, 70)
        self.enable_localization = False
        self.destination = Vector3(UNKNOWN, UNKNOWN, 0)
        self.kick = False
        self.saveimage = False
        self.resetLocalization = False
        self.reset_point = Vector3(UNKNOWN, UNKNOWN, 0)
        self.goalieAttacking = False
        self.timeToReachBall = 9999.0
        self.timeToReachStriker = 9999.0
        self.timeToReachMidFielder = 9999.0
        self.timeToReachDefender = 9999.0
        self.currentRole = None
        # publish node
        self.dmotion_pub = rospy.Publisher('/humanoid/ActionCommand',
                                           ActionCmd, queue_size=1)

    def publish(self):
        """Publish messages."""
        self.dmotion_pub.publish(self.actions)
