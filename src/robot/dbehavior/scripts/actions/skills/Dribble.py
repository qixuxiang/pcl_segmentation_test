"""Dribble skill."""

import rospy
from math import degrees, atan2
from ...misc.bt import Action, see_ball
from ...misc.types.constant import LEFT, ONLY_KICK
from ...misc.types.field_geometry import (attacking_right, attacking_left,
                                          get_attack_goal, facing_goal)
from ...misc.utils.calc_attack_point import get_rub, get_attack_result
from ...misc.utils.timer import Timer
from ...misc.utils.mathutil import get_dis

DRIBBLE_THRES = 30
EPSO = 1e-10

DRIBBLE_SAFE = -20
DRIBBLE_X_B = -2
DRIBBLE_X_AD_B = -3  # 2
DRIBBLE_X_AD_F = 3  # 2
DRIBBLE_X_MID = 3
DRIBBLE_X_TOP = 4

DRIBBLE_Y_B = 1
DRIBBLE_Y_AD_B = 2
DRIBBLE_Y_AD_F = 2
DRIBBLE_Y_MID = 1
DRIBBLE_Y_TOP = EPSO

STEP_L = 12
STEP_R = -12
RM_L = 3.5
RM_R = -15.5
LM_L = 15.5
LM_R = -3.5

x_can = [DRIBBLE_X_B, DRIBBLE_X_B, DRIBBLE_X_AD_B, 0, DRIBBLE_X_AD_F,
         DRIBBLE_X_MID, DRIBBLE_X_TOP, DRIBBLE_X_TOP]
y_can = [EPSO, DRIBBLE_Y_B, DRIBBLE_Y_B, DRIBBLE_Y_AD_B, DRIBBLE_Y_AD_F,
         DRIBBLE_Y_MID, DRIBBLE_Y_TOP, EPSO]
rec_can = [x_can[i] * 1.0 / y_can[i] for i in range(0, len(x_can))]

DEBUG_DRIBBLE = True


# todo, small dribble and large dribble
@see_ball
class Dribble(Action):
    """Dribble."""

    def init(self):
        """Init."""
        self.sx, self.sy = 0, 0
        self.exit_cycle = 0
        self.exit_cycle2 = 0
        self.prev_angle = 0
        self.started = False
        self.cycle = 0
        self.timer = Timer()

    def reset_(self):
        """Reset."""
        self.cycle = 0
        self.started = False
        self.exit_cycle = 0
        self.exit_cycle2 = 0

    def tick(self):
        """Tick."""
        rospy.logdebug('Dribble tick')
        # todo, don't quit and approach_ball, which is very slow
        self.lookat(0, 60)
        self.cycle += 1

        if self.cfg.ATTACK_MODE is ONLY_KICK and self.cfg.KICK_ABILITY:
            return self.success()

        # self.log_error()

        if not self.started:
            self.timer.restart()

        if not self.world.see_ball:
            self.step()
            return self.failure()

        # self.world.check_goal = True
        # print '----------- Dribble -----------------'
        # print 'enable checkgoal ', self.world.check_goal
        #
        # return self.failure()
        #

        # la = attacking_left() and -250 < self.world.robot_pos.x < -150 and \
        #     abs(self.world.field_angle) > 100
        # lr = attacking_right() and 250 > self.world.robot_pos.x > 150 and \
        #     abs(self.world.field_angle) < 80

        # print self.world.check_goal_timer.elapsed()
        # if not self.near_goal() and \
        #     self.world.check_goal_timer.elapsed() > 8000000 and (la or lr):
        #     rospy.loginfo('Enable check goal')
        #
        #     self.world.check_goal = True
        #     self.world.check_goal_timer.restart()
        #     return self.failure()

        rub = get_rub()
        angle = degrees(atan2(rub.y, rub.x))
        robot_pos = self.world.robot_pos

        if not DEBUG_DRIBBLE and abs(robot_pos.y) > 100 and \
                not facing_goal(self.world.robot_pos):
            if abs(robot_pos.x) < 350:

                if abs(robot_pos.x) < 300 and abs(angle) > 50 and \
                        abs(self.prev_angle) > 15:
                    self.exit_cycle += 1
                    if self.exit_cycle > 5:
                        self.step()
                        print 'Dribble exit 1'
                        return self.failure()

                elif abs(robot_pos.x) > 300 and abs(angle) > 10 and \
                        abs(self.prev_angle) > 10:
                    self.exit_cycle += 1
                    if self.exit_cycle > 5:
                        print 'Dribble exit 2'
                        self.step()
                        return self.failure()

        # elif self.near_goal():
        #     final_dest, _, _ = get_attack_result()
        #     diff = final_dest.anglez - self.world.field_angle
        #
        #     if attacking_left():
        #         y = self.world.robot_pos.y
        #         if -100 < y < -70:  # near left goal
        #             if diff > 0:
        #                 print 'l1'
        #                 return self.failure()
        #         elif 70 < y < 100:  # near right goal
        #             if diff < 0:
        #                 print 'l2'
        #                 return self.failure()
        #
        #         # if abs(self.world.field_angle) < 145:
        #         #     print 'Dribble exit 3'
        #         #     return self.failure()
        #     else:
        #         y = self.world.robot_pos.y
        #         if -100 < y < -70:
        #             if diff < 0:
        #                 print 'r1'
        #                 return self.failure()
        #         elif 70 < y < 100:
        #             if diff > 0:
        #                 print 'r2'
        #                 return self.failure()

                # if abs(self.world.field_angle) > 45:
                #     print 'Dribble exit 4'
                #     return self.failure()

        # if self.near_goal():
        #     field_angle = self.world.field_angle
        #     kick_direction = self.gc.kick_direction
        #
        #     if kick_direction is LEFT:
        #         if field_angle

        if self.enable_kick():
            self.world.enable_kick = True
            return self.success()

        self.prev_angle = angle

        vy = self.world.vy
        if vy <= 0:
            current_l = STEP_L - (STEP_L - RM_L) / 1.8 * abs(vy)
            current_r = STEP_R - (STEP_R - RM_R) / 1.8 * abs(vy)
        else:
            current_l = STEP_L - (STEP_L - LM_L) / 1.8 * abs(vy)
            current_r = STEP_R - (STEP_R - LM_R) / 1.8 * abs(vy)

        eye_y = (current_l + current_r) / 2.0

        ball_y = self.world.ball_field.y
        ball_x = self.world.ball_field.x
        diff_y = ball_y - eye_y

        if (not self.near_goal() and STEP_L > diff_y > STEP_R) or \
                (self.near_goal() and STEP_L * 1.5 > diff_y > STEP_R * 1.5):
            # todo degbug here
            pass
        else:
            self.exit_cycle2 += 1

        if self.exit_cycle2 > 0:
            self.step()
            rospy.loginfo('Dribble exit 3')
            return self.failure()
        else:
            diff_x = ball_x + 10  # FIXME

            theta = diff_x / abs(diff_y + 0.00001)

            for i in range(0, 7):
                if rec_can[i] <= theta <= rec_can[i + 1]:
                    a_s = (theta * y_can[i] - x_can[i]) / (
                        x_can[i + 1] - x_can[i] -
                        theta * (y_can[i + 1] - y_can[i]))
                    self.sx = x_can[i] + a_s * (x_can[i + 1] - x_can[i])
                    self.sy = y_can[i] + a_s * (y_can[i + 1] - y_can[i])
                if diff_x >= 0:
                    self.sx = min(self.sx, diff_x)
                else:
                    self.sx = max(self.sx, diff_x)
                self.sy = min(abs(diff_y), self.sy)
            if diff_y < 0:
                self.sy = -self.sy

            # print self.sy
            # if self.sy < -1.5:
            #     t = -5
            # elif self.sy > 1.5:
            #     t = 5
            # else:
            #     t = 0

            t = 0

            self.walk(self.sx, self.sy, t)
            return self.running()

    def log_error(self):
        """Log error."""
        if self.near_goal() and self.world.see_unknown_goal:
            rospy.loginfo('see unknown goal near goal')

    def enable_kick(self):
        """Enable Kick."""
        # return True
        # return False
        # return True
        if not self.cfg.KICK_ABILITY:
            return False

        final_dest, _, _ = get_attack_result()
        # diff = final_dest.anglez - self.world.field_angle

        ENEMY_GOAL = get_attack_goal(self.cfg.GOAL_SHIFT)

        if self.cfg.KICK_RANGE[0] < get_dis(ENEMY_GOAL, self.world.robot_pos) \
                < self.cfg.KICK_RANGE[1]:
            self.world.enable_kick = True
            return True
        elif self.gc.kickoff and self.gc.secondaryTime > 8:
            self.world.enable_kick = True
            return True
        else:
            return False

    def near_goal(self):
        """Near goal."""
        if self.inside_goal():
            return True

        ENEMY_GOAL = get_attack_goal(self.cfg.GOAL_SHIFT)

        # angle = self.world.field_angle
        # if attacking_right() and abs(angle) > 70:
        #     return False
        # elif attacking_left() and abs(angle) < 120:
        #     return False

        if get_dis(ENEMY_GOAL, self.world.robot_pos) < 125:
            return True
        else:
            return False

    def inside_goal(self):
        """Inside goal."""
        robot_pos = self.world.robot_pos
        angle = self.world.field_angle
        if attacking_left() and robot_pos.x < -400 and \
                abs(robot_pos.y) < 90 and abs(angle) > 130:
            return True
        elif attacking_right() and robot_pos.x > 400 and \
                abs(robot_pos.y) < 90 and abs(angle) < 50:
            return True

# if didn't see goal recently, don't dribble to a very large angle
