"""BlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:40:48+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: blackboard.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:19+08:00
# @Copyright: ZJUDancer

from dmotion.msg import MotionShareData
from dvision.msg import VisionShareData
from geometry_msgs.msg import Vector3
from .status_bb import StatusBlackBoard
from .params_bb import ParamsBlackBoard
from .receiver_bb import ReceiverBlackBoard
# from .gc_bb import GameControllerBlackBoard
from ..types.constant import UNKNOWN


class BlackBoard(StatusBlackBoard):
    """BlackBoard for subscribing and publishing robot status."""

    def __init__(self):
        """Init BlackBoard."""
        super(BlackBoard, self).__init__()
        # self.gc = GameControllerBlackBoard()
        # Motion
        self.motion = MotionShareData()
        self.motion.robotCtrl = Vector3(0, 0, 0)
        self.motion.fieldAngle = 0
        self.motion.curPlat = None
        self.motion.vy = None
        self.motion.stable = None
        self.motion.lower_board_connected = False
        self.motion.deltaData = Vector3(0, 0, 0)
        # TODO(corenel) which type? rospy.Time.Duration or uint64?
        self.motion.uptime = 0
        # Vision
        self.vision = VisionShareData()
        self.vision.see_ball = False
        self.vision.see_circle = False
        self.vision.see_goal = False
        self.see_both_goal = False
        self.vision.see_unknown_goal = False
        self.vision.ball_global = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.ball_field = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.ball_image = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.left_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.right_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.unknown_goal = Vector3(UNKNOWN, UNKNOWN, 0)
        self.vision.robot_pos = Vector3(UNKNOWN, UNKNOWN, 0)
        # Params
        self.params = ParamsBlackBoard()
        # Receiver
        self.receiver = ReceiverBlackBoard()
