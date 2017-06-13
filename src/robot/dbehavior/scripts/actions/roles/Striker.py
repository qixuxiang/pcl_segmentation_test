"""Striker role."""
from geometry_msgs.msg import Vector3
from .Role import Role
from ..skills.Attack import Attack
from ..skills.SeekBall import SeekBall
from ..headskill.ScanField import ScanField
from ...misc.bt import sequence, ev_sequence
from ...misc.bt import Action
from ...misc.types.constant import (LEFT, RIGHT, ROLE_MIDFIELDER,
                                    ROLE_STRIKER, X, Y)
from ...misc.types.field_geometry import (get_attack_goal, set_attack_target,
                                          LEFT_ATTACK_POS_DOWN,
                                          RIGHT_ATTACK_POS_DOWN,
                                          HALF_FIELD_LENGTH,
                                          LEFT_ATTACK_POS_UP,
                                          PENALTY_X,
                                          attacking_left,
                                          attacking_right)
# if ball is at our half, and there is no obstacle, then kick
# Brain class setting attack target
from ...misc.utils.mathutil import get_dis, get_magnitude

THRE = 100


class StrikerBrain(Action):
    """StrikerBrain."""

    def tick(self):
        """Tick."""
        kick_direction = self.gc.kick_direction

        # if abs(self.world.robot_pos.x) < PENALTY_X / 4 * 3
        # if kick_direction is LEFT:
        #     print 'left up'
        #     set_attack_target(LEFT_ATTACK_POS_UP)
        # elif kick_direction is RIGHT:
        #     print 'right down'
        #     set_attack_target(RIGHT_ATTACK_POS_DOWN)
        # else:
        #     print 'enemy goal'
        #

        # on kick off
        # if self.gc.kickoff and self.gc.secondaryTime > 1 and \
        #         abs(self.world.robot_pos.x) < 100:
        #     if kick_direction is LEFT:
        #         set_attack_target(LEFT_ATTACK_POS_UP)
        #     else:
        #         set_attack_target(RIGHT_ATTACK_POS_DOWN)
        #
        # else:

        enemy_goal = get_attack_goal(self.cfg.GOAL_SHIFT)

        if abs(self.world.ball_field.x) < 250:
            y = self.world.robot_pos.y

            k = 1

            if abs(y) < THRE:
                enemy_goal.y -= y * k
            elif y > THRE:
                enemy_goal.y -= THRE
            elif y < -THRE:
                enemy_goal.y += THRE

            if self.near_goal():
                if kick_direction is LEFT:
                    enemy_goal.x -= 50
                else:
                    enemy_goal.x += 50

        # if abs(self.world.robot_pos.y) < 80:
        #     enemy_goal.y = self.world.robot_pos.y

        set_attack_target(enemy_goal)
        # print enemy_goal

        # enemy_goal = get_attack_goal(self.cfg.GOAL_SHIFT)
        # set_attack_target(enemy_goal)

        return self.success()

    def near_goal(self):
        """Near goal."""
        if self.inside_goal():
            return True

        ENEMY_GOAL = get_attack_goal(self.cfg.GOAL_SHIFT)
        if get_dis(ENEMY_GOAL, self.world.robot_pos) < 125:
            return True
        else:
            return False

    def inside_goal(self):
        """Inside goal."""
        robot_pos = self.world.robot_pos
        # angle = self.world.field_angle
        if attacking_left() and robot_pos.x < -400 and \
                abs(robot_pos.y) < 100:
            return True
        elif attacking_right() and robot_pos.x > 400 and \
                abs(robot_pos.y) < 100:
            return True


sc = ScanField()


class _Striker(Role):
    def tick(self):
        sc.tick()

        if not self.world.lower_board_connected:
            self.gc.secsSinceUnpenalised.restart()

        if self.world.reentry and not self.world.see_ball:
            if attacking_left():
                self.goto(Vector3(X, Y, 179))

                if self.got_dest(Vector3(X, Y, 179)):
                    self.world.reentry = False
            else:
                self.goto(Vector3(X, Y, 0))
                if self.got_dest(Vector3(X, Y, 0)):
                    self.world.reentry = False
            print '_Striker running 2'
            return self.running()

        elif not self.world.see_ball and \
            ((self.gc.secsSincePlay.elapsed() < 15000000 and
              self.gc.connected) or
             self.gc.secsSinceUnpenalised.elapsed() < 15000000):
            if attacking_left():
                self.goto(Vector3(75, 0, 179))
            else:

                self.goto(Vector3(-75, 0, 0))
            print '_Striker running 1'
            return self.running()

        else:
            return self.success()


Striker = ev_sequence(StrikerBrain,
                      _Striker,
                      sequence(SeekBall, Attack))

# Striker = ev_sequence(StrikerBrain, SeekBall, Attack)
