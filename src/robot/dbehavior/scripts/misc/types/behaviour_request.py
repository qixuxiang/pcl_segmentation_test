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
from dbehavior.msg import BehaviourShareData, TeamShareData
from ..types.constant import UNKNOWN
from ..types.action_command import crouch, head


class BehaviourRequest(object):
    """BehaviourRequest."""

    def __init__(self):
        """Init."""
        super(BehaviourRequest, self).__init__()
        # action command
        # self.update_gait_vec = False
        self.actions = ActionCmd()
        crouch(self.actions)
        head(self.actions, 0, 70)
        # behaviour request
        self.behaviour = BehaviourShareData()
        self.behaviour.enable_localization = False
        self.behaviour.destination = Vector3(UNKNOWN, UNKNOWN, 0)
        self.behaviour.kick = False
        self.behaviour.save_image = False
        self.behaviour.reset_localization = False
        self.behaviour.reset_point = Vector3(UNKNOWN, UNKNOWN, 0)
        # team
        self.team = TeamShareData()
        self.team.goalie_attacking = False
        self.team.time_to_reach_ball = 9999.0
        self.team.time_to_reach_striker = 9999.0
        self.team.time_to_reach_midfielder = 9999.0
        self.team.time_to_reach_defender = 9999.0
        self.team.current_role = None
        # publish node
        self.actions_pub = rospy.Publisher('/humanoid/ActionCommand',
                                           ActionCmd, queue_size=1)
        self.behaviour_pub = rospy.Publisher('/humanoid/BehaviourShareData',
                                             BehaviourShareData, queue_size=1)
        self.team_pub = rospy.Publisher('/humanoid/TeamShareData',
                                        TeamShareData, queue_size=1)

    def publish(self):
        """Publish messages."""
        self.actions_pub.publish(self.actions)
        self.behaviour_pub.publish(self.behaviour)
        self.team_pub.publish(self.team)