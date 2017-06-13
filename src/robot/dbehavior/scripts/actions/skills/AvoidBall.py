"""AvoidBall skill."""

from math import degrees, cos, sin
from geometry_msgs.msg import Vector3
from ..headskills.TrackBall import TrackBall
from ...misc.bt import Action, parallel, see_ball
from ...misc.types.FieldGeometry import get_attack_goal
from ...misc.types.VecPos import VecPos
from ...misc.utils.CalcAttackPoint import get_rub
from ...misc.utils.calc_walk_engine import get_walk
from ...misc.utils.mathutil import (angle_between, PI_2, get_dis,
                                    abs_angle_diff, get_magnitude)

# Avoid
AVOID_DIS = 100
AVOID_POINT_DOWN = VecPos(0, -AVOID_DIS * 1.5)
AVOID_POINT_UP = VecPos(0, AVOID_DIS * 1.5)


@see_ball
class _AvoidBall(Action):
    """AvoidBall."""

    def tick(self):
        """Tick."""
        destination = self.calc_avoid_point()

        if not self.got_dest(destination):
            # print 'avoid ball go to dest'
            # diff_angle = abs(self.world.field_angle - destination.anglez)
            dis = get_dis(self.world.ball_global, self.world.robot_pos)
            if dis < AVOID_DIS:
                x, y, t = get_walk(destination, self.world.robot_pos, True)
            else:
                x, y, t = get_walk(destination, self.world.robot_pos, False)
            self.req.destination = destination

            self.walk(x, y, t)
            return self.running()
        else:
            # print 'avoid ball get dest'
            self.crouch()
            return self.success()

    def calc_avoid_point(self):
        """Calculate aviod point."""
        ball_pos = self.world.ball_global
        goal = get_attack_goal(self.cfg.GOAL_SHIFT)
        theta = angle_between(goal, ball_pos)

        rub = get_rub()

        if rub.y > 0:
            final_x = ball_pos.x + AVOID_POINT_UP.x * cos(theta) - \
                AVOID_POINT_UP.y * sin(theta)
            final_y = ball_pos.y + AVOID_POINT_UP.x * sin(theta) + \
                AVOID_POINT_UP.y * cos(theta)

            return Vector3(final_x, final_y, degrees(theta - PI_2))
        else:
            final_x = ball_pos.x + AVOID_POINT_DOWN.x * cos(theta) - \
                AVOID_POINT_DOWN.y * sin(theta)
            final_y = ball_pos.ya + AVOID_POINT_DOWN.x * sin(theta) + \
                AVOID_POINT_DOWN.y * cos(theta)
            return Vector3(final_x, final_y, degrees(theta + PI_2))

    def got_dest(self, pos):
        """Check if self has got to destination."""
        dis = get_dis(pos, self.world.robot_pos)
        dis_ball = get_magnitude(self.world.ball_field)
        diff_angle = abs_angle_diff(pos.anglez - self.world.robot_pos.anglez)

        if dis < 30 and diff_angle < 16 and dis_ball > AVOID_DIS / 2:
            return True
        else:
            return False


AvoidBall = parallel(_AvoidBall,
                     TrackBall)
