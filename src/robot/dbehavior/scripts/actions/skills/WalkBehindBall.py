"""WalkBehindBall skill."""

from math import degrees, atan2, radians, cos, sin

from misc.bt import Action, see_ball
from misc.types.vec_pos import VecPos
from misc.utils.mathutil import angle_normalization, get_dis, abs_angle_diff
from misc.utils.calc_attack_point import get_attack_result, get_rub
from misc.utils.calc_walk_engine import get_walk_field

SAFE_DIST = 30
ANGLE_PER_TICK = 20
CLOCK_WISE = -1
ANTI_CLOCK_WISE = 1


@see_ball
class WalkBehindBall(Action):
    """WalkBehindBall."""

    def tick(self):
        """Tick."""
        print 'walk behind ball'
        # stable head
        # todo, very large h
        self.lookat(0, 65)

        ball = self.world.ball_field
        final_dest, _, rub = get_attack_result()

        if not self.world.see_ball:
            return self.failure()

        if self.check_get_atk_dest(final_dest):
            return self.success()
        else:
            if rub.y > 0:
                direction = CLOCK_WISE
            else:
                direction = ANTI_CLOCK_WISE
            # direction = CLOCK_WISE

            r = SAFE_DIST
            theta = ANGLE_PER_TICK * direction
            # print theta

            beta = angle_normalization(180.0 - degrees(atan2(ball.y, ball.x)))
            alpha = radians(theta - beta)
            r_vec = VecPos(r * cos(alpha), r * sin(alpha))

            des = ball + r_vec
            angle = angle_normalization(alpha + 180)

            x, y, t = get_walk_field(des, angle)

            # if x < 0:
            #     x = 0

            # if direction is ANTI_CLOCK_WISE and t < 0:
            #     t *= 0
            # elif direction is ANTI_CLOCK_WISE and t > 0:
            #     t *= 0

            t *= 0.6

            self.walk(x, y, t)
            return self.running()

    def check_get_atk_dest(self, destination):
        """Check if we have got to destination.

        :param destination: global destination
        """
        # todo, not seen ball, get rub bug
        # rub = get_rub()

        return self.got_dest(destination)

    def got_dest(self, pos):
        """Check if we have got to destination."""
        dis = get_dis(pos, self.world.robot_pos)
        diff_angle = abs_angle_diff(pos.anglez - self.world.robot_pos.anglez)

        if dis < 35 and diff_angle < 15:
            return True
        else:
            return False
