"""Parameters BlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T15:03:34+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: config_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:52+08:00
# @Copyright: ZJUDancer

from .status_bb import StatusBlackBoard
from ..types.constant import Constant
from ..types.robot_config import RobotConfig


class ParamsBlackBoard(StatusBlackBoard):
    """BlackBoard for parameters and robot configs."""

    def __init__(self):
        """Init ParamsBlackBoard."""
        super(ParamsBlackBoard, self).__init__()
        self.constant = Constant()
        self.robot_config = RobotConfig()
        # get params
        self.gparam("/dbehaviour/constant/HALF_TIME",
                    self.constant.HALF_TIME, self)
