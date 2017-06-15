"""MotionBlackBoard class."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-15T15:23:04+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: motion_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-15T15:23:11+08:00
# @Copyright: ZJUDancer

from dmotion.msg import MotionShareData
from geometry_msgs.msg import Vector3


class MotionBlackBoard(MotionShareData):
    """BlackBoard for robot motion messages."""

    def __init__(self):
        """Init default value."""
        super(MotionBlackBoard, self).__init__()
        self.robotCtrl = Vector3(0, 0, 0)
        self.fieldAngle = 0
        self.curPlat = None
        self.vy = None
        self.stable = None
        self.lower_board_connected = False
        self.deltaData = Vector3(0, 0, 0)
        # TODO(corenel) which type? rospy.Time.Duration or uint64?
        self.uptime = 0
