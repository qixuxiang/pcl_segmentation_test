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
from ..types.robot_config import RobotConfig
from ..types.vec_pos import VecPos
from ..utils.mathutil import degree_between


class ParamsBlackBoard(StatusBlackBoard):
    """BlackBoard for parameters and robot configs."""

    def __init__(self):
        """Init ParamsBlackBoard."""
        super(ParamsBlackBoard, self).__init__()
        # init member varibales
        self.player_number = None
        self.player_team = None
        self.skill = None
        self.robot_config = RobotConfig()

        # get general info
        self.gparam("/dbehaviour/player/number", "player_number", self)
        self.gparam("/dbehaviour/player/team", "player_team", self)
        self.gparam("/dbehaviour/skill", "skill", self)

        # get robot config
        robot_id = self.player_number
        self.gparam("/dbehaviour/robot_{}/FINAL_ATK_DIS".format(robot_id),
                    "FINAL_ATK_DIS", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/LEFT_KICK".format(robot_id),
                    "LEFT_KICK", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/LEFT_KICK_POINT"
                           .format(robot_id),
                           "LEFT_KICK_POINT", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/RIGHT_KICK".format(robot_id),
                    "RIGHT_KICK", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/RIGHT_KICK_POINT"
                           .format(robot_id),
                           "RIGHT_KICK_POINT", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_ABILITY".format(robot_id),
                    "KICK_ABILITY", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_RANGE".format(robot_id),
                    "KICK_RANGE", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_OFF".format(robot_id),
                    "KICK_OFF", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT".format(robot_id),
                           "DOGE_POINT", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT_UP"
                           .format(robot_id),
                           "DOGE_POINT_UP", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT_DOWN"
                           .format(robot_id),
                           "DOGE_POINT_DOWN", self.robot_config)
        self.doge_angle = degree_between(self.DOGE_POINT, VecPos(0, 50))
        self.gparam("/dbehaviour/robot_{}/LINE_UP_TIMEOUT".format(robot_id),
                    "LINE_UP_TIMEOUT", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/DEST_REGION".format(robot_id),
                    "DEST_REGION", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/DEST_RE_ANGLE".format(robot_id),
                    "DEST_RE_ANGLE", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/ATTACK_MODE".format(robot_id),
                    "ATTACK_MODE", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/MANUAL_SET".format(robot_id),
                    "MANUAL_SET", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/MANUAL_SET_POSITION"
                    .format(robot_id),
                    "MANUAL_SET_POSITION", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/GOAL_SHIFT".format(robot_id),
                    "GOAL_SHIFT", self.robot_config)
