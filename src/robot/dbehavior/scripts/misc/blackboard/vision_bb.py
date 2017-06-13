"""VisionBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:57:57+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: vision_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:37+08:00
# @Copyright: ZJUDancer

from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from .status_bb import StatusBlackBoard
from ..types.constant import UNKNOWN
from ..types.vec_pos import VecPos


class VisionBlackBoard(StatusBlackBoard):
    """BlackBoard for robot vision status."""

    def __init__(self):
        """Init VisionBlackBoard."""
        super(VisionBlackBoard, self).__init__()
        self.ball_field = VecPos(UNKNOWN, UNKNOWN)
        self.ball_img = VecPos(UNKNOWN, UNKNOWN)
        self.see_both_goal = False
        self.see_unknown_goal = False
        self.left_goal = VecPos(UNKNOWN, UNKNOWN)
        self.right_goal = VecPos(UNKNOWN, UNKNOWN)
        self.unknown_goal = VecPos(UNKNOWN, UNKNOWN)
        self.height = None
        # localization
        self.robotPos = Vector3(0, 0, 0)
        self.ballPos = VecPos(UNKNOWN, UNKNOWN)
        # recognition
        self.see_ball = False
        self.ballest = VecPos(UNKNOWN, UNKNOWN)
        self.ballest_global = VecPos(UNKNOWN, UNKNOWN)
        self.est_goal_left = VecPos(UNKNOWN, UNKNOWN)
        self.est_goal_right = VecPos(UNKNOWN, UNKNOWN)
        self.est_goal_unknown = VecPos(UNKNOWN, UNKNOWN)
        self.obstacle = VecPos(UNKNOWN, UNKNOWN)

        # subscribe
        self.subscribe('/dvision/ball', String, self, 'ball')
