"""class BehaviourRequest."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-13T10:43:41+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: behaviour_request.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-13T10:43:52+08:00
# @Copyright: ZJUDancer

from geometry_msgs.msg import Vector3
from dmotion.msg import ActionCmd
from ..types.constant import UNKNOWN


class BehvaviourRequest(object):
    """BehaviourRequest."""

    def __init__(self):
        """Init."""
        super(BehvaviourRequest, self).__init__()
        self.update_gait_vec = False
        self.actions = None
        self.enable_localization = False
        self.destination = Vector3(UNKNOWN, UNKNOWN, 0)
        self.kick = False
        self.saveimage = False
        self.resetLocalization = False
        self.reset_point = Vector3(UNKNOWN, UNKNOWN, 0)
