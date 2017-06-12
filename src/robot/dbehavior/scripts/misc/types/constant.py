"""class Constant."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T13:04:50+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: constant.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T13:27:43+08:00
# @Copyright: ZJUDancer


class Constant(object):
    """Constant in GC or dmotion."""

    def __init__(self):
        """Init."""
        super(Constant, self).__init__()

        self.HALF_TIME = None
        self.BALL_DROP_CYCLE = None
        self.LOST_BALL_DELAY = None
        self.MAX_VIEW_DIST = None
        self.UNKNOWN = None
        self.MAX_PLAT_YAW = None
        self.MAX_PLAT_PITCH = None
        self.MIN_PLAT_PITCH = None
        # game controller state
        self.STATE_INITIAL = None
        self.STATE_READY = None
        self.STATE_SET = None
        self.STATE_PLAYING = None
        self.STATE_FINISHED = None
        self.STATE_INVALID = None
        self.STATE_PENALISED = None
        self.ROBOTS_PER_TEAM = None
        self.TEAM_BLUE = None
        self.TEAM_CYAN = None
        self.TEAM_RED = None
        self.TEAM_MAGENTA = None
        # Stable
        self.STABLE = None
        self.RIGHTDOWN = None
        self.LEFTDOWN = None
        self.FRONTDOWN = None
        self.BACKDOWN = None
        # speed
        self.WALK_SPEED = None
        self.TURN_SPEED = None
        # Role enums
        self.ROLE_NONE = None
        self.ROLE_GOALIE = None
        self.ROLE_STRIKER = None
        self.ROLE_DEFENDER = None
        self.ROLE_MIDFIELDER = None
        # image
        self.IMAGE_WIDTH = None
        self.IMAGE_HEIGHT = None
        self.VIEW_CENTER = None
        # Attack side
        self.INVALID = None
        self.LEFT = None
        self.RIGHT = None
        # ball
        self.NUM_LOST_FRAMES_TEAM_BALL_LOST = None
        self.NUM_LOST_FRAMES_CAN_SEE_BALL = None
        # ATTACK MODE
        self.ONLY_KICK = None
        self.KICK_NEAR_GOAL = None
        # roles
        self.GOALIE_ID = None
        self.GOALIE = None
        self.STRIKER = None
        self.DEFENDER = None
        self.KICKOFF = None
        # pos
        self.X = None
        self.Y = None

        def get_role_str(self, role):
            if role is self.ROLE_NONE:
                return 'NONE'
            elif role is self.ROLE_GOALIE:
                return 'Goalie'
            elif role is self.ROLE_STRIKER:
                return 'Striker'
            elif role is self.ROLE_DEFENDER:
                return 'Defender'
            elif role is self.ROLE_MIDFIELDER:
                return 'MidFielder'
            else:
                return 'Role id invalid!'
