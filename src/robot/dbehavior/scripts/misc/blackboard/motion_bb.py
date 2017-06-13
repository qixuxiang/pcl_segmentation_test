"""MotionBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:58:08+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: motion_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:09:02+08:00
# @Copyright: ZJUDancer

from geometry_msgs.msg import Vector3
from .status_bb import StatusBlackBoard


class MotionBlackBoard(StatusBlackBoard):
    """BlackBoard for robot motion status."""

    def __init__(self):
        """Init MotionBlackBoard."""
        super(MotionBlackBoard, self).__init__()

        self.robotCtrl = None
        self.fieldAngle = 0
        self.eular = None
        self.curPlat = None
        self.compass = None
        self.vy = None
        self.gyro = None
        self.stable = None
        self.lower_board_connected = False
        self.deltaData = Vector3(0, 0, 0)
        self.actions = None
        self.uptime = 0
