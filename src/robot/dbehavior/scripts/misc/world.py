"""World for the global information of the game."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T15:32:53+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: world.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T15:33:25+08:00
# @Copyright: ZJUDancer

import rospy
from ..utils.mathutil import calc_global_position
from ..utils.timer import Timer, get_current_time
from ..types.vec_pos import VecPos
from ..types.constant import UNKNOWN, ROLE_NONE
from ..types.field_geometry import inside_field, inside_view


class World(object):
    """The World object is used to save useful information about the game."""

    def __init__(self, blackboard):
        """Init."""
        self.bb = blackboard
        self.obstalce = VecPos(UNKNOWN, UNKNOWN)
        self.reentry = False
        self.check_goal = False
        self.enable_kick = False
        self.last_seen_ball_field = VecPos(UNKNOWN, UNKNOWN)
        self.last_seen_blue_goal_time = 0
        self.last_seen_red_goal_time = 0
        self.check_goal_timer = Timer()
        self.uptime = 0
        self.ball_field = VecPos(UNKNOWN, UNKNOWN)
        self.ball_global = VecPos(UNKNOWN, UNKNOWN)

        # memorize ball temporally
        self.mem_ball_field = VecPos(UNKNOWN, UNKNOWN)
        self.mem_ball_global = VecPos(UNKNOWN, UNKNOWN)
        # whether memorized ball position is still valid
        # changed by HeadSkill.CheckBall
        self.mem_ball_valid = False

        self.lost_ball = Timer().start()
        self.ball_valid_cycle = 0
        self.lost_ball.elapsedTime = 100000000

        self.last_ball_global = VecPos(UNKNOWN, UNKNOWN)

        # whether we saw goal
        self.lost_goal_cycle = 0
        self.see_both_goal = False
        self.see_unknown_goal = False
        self.left_goal = VecPos(UNKNOWN, UNKNOWN)
        self.right_goal = VecPos(UNKNOWN, UNKNOWN)
        self.unknown_goal = VecPos(UNKNOWN, UNKNOWN)
        self.enable_checkgoal = False

        self.blue_goal_center = VecPos(UNKNOWN, UNKNOWN)
        self.red_goal_center = VecPos(UNKNOWN, UNKNOWN)

        self.role = ROLE_NONE

        # whether we are stable
        self.stable = True

        self.see_ball = False
        self.mem_ball = False
        self.plat = VecPos(0, 0)

        # flag for reentry
        self.enable_reentry = False

    def update(self, _blackboard):
        """Update.

        1. consider ball is hided by obstacle
        2. when ball is near left or right to robot's foot,
        very easily it disappears, so we need to remember it for 2 seconds
        """
        # motion
        self.uptime = _blackboard.motion.uptime
        self.lower_board_connected = _blackboard.motion.lower_board_connected
        self.stable = _blackboard.motion.stable
        self.plat = _blackboard.motion.curPlat
        self.vy = _blackboard.motion.vy
        self.deltaData = _blackboard.motion.deltaData

        # vision
        self.vision_ball_field = _blackboard.vision.ball_field
        self.left_goal = _blackboard.recognition.est_goal_left
        self.right_goal = _blackboard.recognition.est_goal_right
        self.unknown_goal = _blackboard.recognition.est_goal_unknown

        # localization
        self.robot_pos = _blackboard.localization.robotPos
        self.field_angle = _blackboard.motion.fieldAngle

        if inside_view(self.left_goal) and inside_view(self.right_goal):
            self.lost_goal_cycle = 0

            self.see_both_goal = True
            self.see_unknown_goal = False

            center = VecPos((self.left_goal.x + self.right_goal.x) / 2.0,
                            (self.left_goal.y + self.right_goal.y) / 2.0)

            if self.robot_pos.x > 75 or abs(self.field_angle) < 85:
                self.last_seen_red_goal_time = get_current_time()
                self.red_goal_center = center
                # print 'goal {}'.format(center)
                self.red_goal_center_global = \
                    calc_global_position(center, self.robot_pos)

            elif self.robot_pos.x < -75 or abs(self.field_angle) > 95:
                self.last_seen_blue_goal_time = get_current_time()
                self.blue_goal_center = center
                self.blue_goal_center_global = \
                    calc_global_position(center, self.robot_pos)

        elif inside_view(self.unknown_goal):
            self.lost_goal_cycle += 1
            self.update_goal_center()
            self.see_both_goal = False
            self.see_unknown_goal = True

        else:
            self.lost_goal_cycle += 1
            self.update_goal_center()
            self.see_both_goal = False
            self.see_unknown_goal = False

        # vision/recognition
        # todo continue
        self.obstalce = _blackboard.recognition.obstacle

        ball_global = _blackboard.recognition.ballest_global
        ball_field = _blackboard.recognition.ballest

        if inside_field(ball_global) and inside_view(ball_field):
            if False:  # ball_field.x < 0:
                print 'suspicious ball: {}'.format(ball_field)
                self.ball_valid_cycle = 0
            else:
                self.ball_valid_cycle += 1
                self.see_ball = True
                self.ball_field = VecPos(ball_field.x, ball_field.y)
                self.last_seen_ball_field = self.ball_field.copy()

                # m = '{} {}\n'.format(self.ball_field.x, self.ball_field.y)
                # print m
                # ball_log.write(m)
                # ball_log.flush()
                self.ball_global = VecPos(ball_global.x, ball_global.y)

                self.mem_ball_field = self.ball_field
                self.mem_ball_global = self.ball_global
                self.mem_ball_valid = True

                self.lost_ball.restart()

        else:
            self.see_ball = False
            self.ball_valid_cycle = 0
            self.ball_field = VecPos(UNKNOWN, UNKNOWN)
            self.ball_global = VecPos(UNKNOWN, UNKNOWN)

            # update memorized ball position
            if self.mem_ball_valid:
                self.update_mem_ball()
                # print self.mem_ball_field
                self.ball_field = self.mem_ball_field
                self.ball_global = self.mem_ball_global
            else:
                self.mem_ball_field = VecPos(UNKNOWN, UNKNOWN)
                self.mem_ball_global = VecPos(UNKNOWN, UNKNOWN)

        self.log_info()

    def update_mem_ball(self):
        """Update mem_ball."""
        self.mem_ball_field.x -= self.deltaData.x
        self.mem_ball_field.y -= self.deltaData.y
        # print self.mem_ball_field
        self.mem_ball_field.rotate(-self.deltaData.z)
        # print '{} {}'.format(type(self.mem_ball_field), self.robot_pos)
        self.mem_ball_global = calc_global_position(self.mem_ball_field,
                                                    self.robot_pos)

    def update_goal_center(self):
        """Update goal center."""
        self.blue_goal_center.x -= self.deltaData.x
        self.blue_goal_center.y -= self.deltaData.y
        self.blue_goal_center.rotate(-self.deltaData.z)

        self.red_goal_center.x -= self.deltaData.x
        self.red_goal_center.y -= self.deltaData.y
        self.red_goal_center.rotate(-self.deltaData.z)

    def ball_lost_sec(self):
        """Second for ball losing."""
        return self.lost_ball.elapsed() / 1000000.0

    def log_info(self):
        """Log info."""
        rospy.logdebug('Attack: {} {} {}\n'.format(self.field_angle,
                                                   self.robot_pos.x,
                                                   self.robot_pos.y))
