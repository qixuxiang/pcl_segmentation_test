"""VisionBlackBoard class."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-15T15:20:20+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: vision_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-15T15:20:21+08:00
# @Copyright: ZJUDancer

from dvision.msg import VisionInfo
from geometry_msgs.msg import Vector3
from ..types.constant import UNKNOWN


class VisionBlackBoard(VisionInfo):
    """BlackBoard for robot vision messages."""

    def __init__(self):
        """Init default value."""
        super(VisionBlackBoard, self).__init__()
        self.see_ball = False
        self.see_circle = False
        self.see_goal = False
        self.see_both_goal = False
        self.see_unknown_goal = False
        self.ball_global = Vector3(UNKNOWN, UNKNOWN, 0)
        self.ball_field = Vector3(UNKNOWN, UNKNOWN, 0)
        self.ball_image = Vector3(UNKNOWN, UNKNOWN, 0)
        self.left_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.right_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.unknown_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.robot_pos = Vector3(UNKNOWN, UNKNOWN, 0)
