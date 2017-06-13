"""Lineup skill."""

import rospy
from __future__ import division
from ...misc.bt import Action
from ...misc.types.field_geometry import inside_view
from ...misc.utils.timer import Timer
from ...misc.types.constant import LEFT, RIGHT


class Lineup(Action):
    """Lineup."""

    def init(self):
        """Init."""
        self.diff_x = 0
        self.diff_y = 0

        self.sum_x = 0
        self.sum_y = 0

        self.avg_x = 0
        self.avg_y = 0

        self.line_up_cycle = 0
        self.stop_timer = Timer()
        self.entry = False

        self.move_direction = LEFT

    def reset_(self):
        """Reset."""
        self.line_up_cycle = 0
        self.diff_x = 0
        self.diff_y = 0
        self.sum_x = 0
        self.sum_y = 0
        self.entry = False

    def tick(self):
        """Tick."""
        self.debug_log()

        self.lookat(0, 65)

        if not self.world.see_ball or self.team.id in [1, 2, 3]:
            return self.failure()

        if not self.entry:
            # stop 0.5s to get stable ball position
            self.entry = True
            self.stop_timer.restart()
            self.crouch()

            return self.running()

        elif self.stop_timer.elapsed() < self.cfg.LINE_UP_TIMEOUT:

            # ball = self.world.ball_field
            ballvision = self.world.vision_ball_field
            if abs(ballvision.y) > 35 or abs(ballvision.x) > 35:
                return self.failure()

            self.sum_x += ballvision.x
            self.sum_y += ballvision.y
            self.line_up_cycle += 1

            # self.diff_x = ball.x - KICK_POINT.x
            # self.diff_y = ball.y - KICK_POINT.y

            self.avg_x = self.sum_x / self.line_up_cycle
            self.avg_y = self.sum_y / self.line_up_cycle

            self.diff_x = self.avg_x - self.cfg.LEFT_KICK_POINT.x
            self.diff_y = self.avg_y - self.cfg.LEFT_KICK_POINT.y

            self.crouch()
            return self.running()

        else:
            ball = self.world.ball_field
            if abs(ball.x) > 35 or abs(ball.y) > 35:
                return self.failure()
            elif not inside_view(self.world.vision_ball_field):
                rospy.loginfo('[Lineup] stop because vision ball lost')
                return self.failure()

            if not abs(self.diff_x) < 5 or not abs(self.diff_y) < 5:
                delta = self.world.deltaData

                self.diff_x -= delta.x
                self.diff_y -= delta.y

                x, y, t = [0, 0, 0]

                if self.diff_x > 3:
                    x = 2
                elif self.diff_x < -3:
                    x = -2
                else:
                    x = 0

                if self.diff_y > 3:
                    y = 2
                elif self.diff_y < -3:
                    y = -2
                else:
                    y = 0

                self.walk(x, y, t)
                return self.running()

            else:
                self.crouch()
                return self.success()
