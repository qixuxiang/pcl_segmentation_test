"""class RobotConfig."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T13:04:50+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: constant.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T13:27:43+08:00
# @Copyright: ZJUDancer


class RobotConfig(object):
    """RobotConfig."""

    def __init__(self):
        """Init."""
        super(RobotConfig, self).__init__()

        self.FINAL_ATK_DIS = None
        # kick
        self.LEFT_KICK = None
        self.LEFT_KICK_POINT = None
        self.RIGHT_KICK = None
        self.RIGHT_KICK_POINT = None
        self.KICK_RANGE = None
        self.KICK_ABILITY = None
        self.KICK_OFF = None
        # doge
        self.DOGE_POINT = None
        self.DOGE_POINT_UP = None
        self.DOGE_POINT_DOWN = None
        self.doge_angle = None
        # dest
        self.LINE_UP_TIMEOUT = None
        self.DEST_REGION = None
        self.DEST_RE_ANGLE = None
        # attack
        self.ATTACK_MODE = None
        # goal
        self.GOAL_SHIFT = None
        # manual
        self.MANUAL_SET = None
        self.MANUAL_SET_POSITION = None
