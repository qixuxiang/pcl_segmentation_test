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
from ..types.vec_pos import VecPos
from ..utils.mathutil import degree_between


class ParamsBlackBoard(StatusBlackBoard):
    """BlackBoard for parameters and robot configs."""

    def __init__(self, robot_id):
        """Init ParamsBlackBoard."""
        super(ParamsBlackBoard, self).__init__()
        self.constant = Constant()
        self.robot_config = RobotConfig()
        # get constant
        self.gparam("/dbehaviour/constant/HALF_TIME",
                    "HALF_TIME", self.constant)
        self.gparam("/dbehaviour/constant/BALL_DROP_CYCLE",
                    "BALL_DROP_CYCLE", self.constant)
        self.gparam("/dbehaviour/constant/LOST_BALL_DELAY",
                    "LOST_BALL_DELAY", self.constant)
        self.gparam("/dbehaviour/constant/MAX_VIEW_DIST",
                    "MAX_VIEW_DIST", self.constant)
        self.gparam("/dbehaviour/constant/UNKNOWN",
                    "UNKNOWN", self.constant)
        self.gparam("/dbehaviour/constant/MAX_PLAT_YAW",
                    "MAX_PLAT_YAW", self.constant)
        self.gparam("/dbehaviour/constant/MAX_PLAT_PITCH",
                    "MAX_PLAT_PITCH", self.constant)
        self.gparam("/dbehaviour/constant/MIN_PLAT_PITCH",
                    "MIN_PLAT_PITCH", self.constant)
        self.gparam("/dbehaviour/constant/STATE_INITIAL",
                    "STATE_INITIAL", self.constant)
        self.gparam("/dbehaviour/constant/STATE_READY",
                    "STATE_READY", self.constant)
        self.gparam("/dbehaviour/constant/STATE_SET",
                    "STATE_SET", self.constant)
        self.gparam("/dbehaviour/constant/STATE_PLAYING",
                    "STATE_PLAYING", self.constant)
        self.gparam("/dbehaviour/constant/STATE_FINISHED",
                    "STATE_FINISHED", self.constant)
        self.gparam("/dbehaviour/constant/STATE_INVALID",
                    "STATE_INVALID", self.constant)
        self.gparam("/dbehaviour/constant/STATE_PENALISED",
                    "STATE_PENALISED", self.constant)
        self.gparam("/dbehaviour/constant/ROBOTS_PER_TEAM",
                    "ROBOTS_PER_TEAM", self.constant)
        self.gparam("/dbehaviour/constant/TEAM_BLUE",
                    "TEAM_BLUE", self.constant)
        self.gparam("/dbehaviour/constant/TEAM_CYAN",
                    "TEAM_CYAN", self.constant)
        self.gparam("/dbehaviour/constant/TEAM_RED",
                    "TEAM_RED", self.constant)
        self.gparam("/dbehaviour/constant/TEAM_MAGENTA",
                    "TEAM_MAGENTA", self.constant)
        self.gparam("/dbehaviour/constant/STABLE",
                    "STABLE", self.constant)
        self.gparam("/dbehaviour/constant/RIGHTDOWN",
                    "RIGHTDOWN", self.constant)
        self.gparam("/dbehaviour/constant/LEFTDOWN",
                    "LEFTDOWN", self.constant)
        self.gparam("/dbehaviour/constant/FRONTDOWN",
                    "FRONTDOWN", self.constant)
        self.gparam("/dbehaviour/constant/BACKDOWN",
                    "BACKDOWN", self.constant)
        self.gparam("/dbehaviour/constant/WALK_SPEED",
                    "WALK_SPEED", self.constant)
        self.gparam("/dbehaviour/constant/TURN_SPEED",
                    "TURN_SPEED", self.constant)
        self.gparam("/dbehaviour/constant/ROLE_NONE",
                    "ROLE_NONE", self.constant)
        self.gparam("/dbehaviour/constant/ROLE_GOALIE",
                    "ROLE_GOALIE", self.constant)
        self.gparam("/dbehaviour/constant/ROLE_STRIKER",
                    "ROLE_STRIKER", self.constant)
        self.gparam("/dbehaviour/constant/ROLE_DEFENDER",
                    "ROLE_DEFENDER", self.constant)
        self.gparam("/dbehaviour/constant/ROLE_MIDFIELDER",
                    "ROLE_MIDFIELDER", self.constant)
        self.gparam("/dbehaviour/constant/IMAGE_WIDTH",
                    "IMAGE_WIDTH", self.constant)
        self.gparam("/dbehaviour/constant/IMAGE_HEIGHT",
                    "IMAGE_HEIGHT", self.constant)
        self.constant.VIEW_CENTER = VecPos(self.constant.IMAGE_WIDTH / 2,
                                           self.constant.IMAGE_HEIGHT / 2)
        self.gparam("/dbehaviour/constant/INVALID",
                    "INVALID", self.constant)
        self.gparam("/dbehaviour/constant/LEFT",
                    "LEFT", self.constant)
        self.gparam("/dbehaviour/constant/RIGHT",
                    "RIGHT", self.constant)
        self.gparam("/dbehaviour/constant/ONLY_KICK",
                    "ONLY_KICK", self.constant)
        self.gparam("/dbehaviour/constant/KICK_NEAR_GOAL",
                    "KICK_NEAR_GOAL", self.constant)
        self.gparam("/dbehaviour/constant/NUM_LOST_FRAMES_TEAM_BALL_LOST",
                    "NUM_LOST_FRAMES_TEAM_BALL_LOST", self.constant)
        self.gparam("/dbehaviour/constant/NUM_LOST_FRAMES_CAN_SEE_BALL",
                    "NUM_LOST_FRAMES_CAN_SEE_BALL", self.constant)
        self.gparam("/dbehaviour/constant/GOALIE_ID",
                    "GOALIE_ID", self.constant)
        self.gparam("/dbehaviour/constant/GOALIE",
                    "GOALIE", self.constant)
        self.gparam("/dbehaviour/constant/STRIKER",
                    "STRIKER", self.constant)
        self.gparam("/dbehaviour/constant/DEFENDER",
                    "DEFENDER", self.constant)
        self.gparam("/dbehaviour/constant/KICKOFF",
                    "KICKOFF", self.constant)
        self.gparam("/dbehaviour/constant/X",
                    "X", self.constant)
        self.gparam("/dbehaviour/constant/Y",
                    "Y", self.constant)
        # get robot config
        self.gparam("/dbehaviour/robot_{}/FINAL_ATK_DIS".format(robot_id),
                    "FINAL_ATK_DIS", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/LEFT_KICK".format(robot_id),
                    "LEFT_KICK", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/LEFT_KICK_POINT".format(robot_id),
                           "LEFT_KICK_POINT", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/RIGHT_KICK".format(robot_id),
                    "RIGHT_KICK", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/RIGHT_KICK_POINT".format(robot_id),
                           "RIGHT_KICK_POINT", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_ABILITY".format(robot_id),
                    "KICK_ABILITY", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_RANGE".format(robot_id),
                    "KICK_RANGE", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/KICK_OFF".format(robot_id),
                    "KICK_OFF", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT".format(robot_id),
                           "DOGE_POINT", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT_UP".format(robot_id),
                           "DOGE_POINT_UP", self.robot_config)
        self.gparam_vecpos("/dbehaviour/robot_{}/DOGE_POINT_DOWN".format(robot_id),
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
        self.gparam("/dbehaviour/robot_{}/MANUAL_SET_POSITION".format(robot_id),
                    "MANUAL_SET_POSITION", self.robot_config)
        self.gparam("/dbehaviour/robot_{}/GOAL_SHIFT".format(robot_id),
                    "GOAL_SHIFT", self.robot_config)
