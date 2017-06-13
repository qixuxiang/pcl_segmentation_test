"""ApproachBall skill."""

import rospy
from .WalkBehindBall import WalkBehindBall
from ..headskills.TrackBall import TrackBall
from misc.bt import Action, parallel, selector, see_ball
from misc.types.field_geometry import (get_attack_goal, attacking_left,
                                          attacking_right)
from misc.types.vec_pos import VecPos
from misc.utils.calc_attack_point import get_attack_result
from misc.utils.calc_walk_engine import get_walk
from misc.utils.mathutil import (get_dis, abs_angle_diff,
                                    calc_field_position,
                                    get_magnitude)

# walk behind ball dis
WALK_BEHIND_BALL_DIS = 20


@see_ball
class _Approach(Action):
    """ApproachBall."""

    def init(self):
        """Init."""
        self.get_dest_cycle = 0

    def reset_(self):
        """Reset."""
        self.get_dest_cycle = 0

    def tick(self):
        """Tick."""
        self.debug_log()

        final_dest, dest, rub = get_attack_result()

        if self.get_dest(final_dest):
            self.step()
            rospy.loginfo('approach ball success')
            return self.success()

        elif -30 < rub.x < 0 and -30 < rub.y < 30:
            return self.failure()

        else:
            if dest is not final_dest:
                dest_field = calc_field_position(final_dest,
                                                 self.world.robot_pos)
                if dest_field.x < 0:
                    dest = final_dest

            self.req.destination = dest
            x, y, t = get_walk(dest, self.world.robot_pos)

            # angle = degree_between(self.world.ball_global,
            #                        self.world.robot_pos)
            # diff = angle_normalization(self.world.field_angle - angle + 180)
            dis = get_magnitude(self.world.ball_field)

            if not dis > 200:
                if rub.x < 10:   # and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 5
                    else:
                        t += 5

                elif rub.x > 10:  # and abs(diff) < 50:
                    if rub.y < 0:
                        t -= 3
                    else:
                        t += 3

            # todo, fix this
            # print 'walk {} {} {}'.format(x, y, t)
            self.walk(x, y, t)

            return self.running()

    def get_dest(self, dest):
        """Get destination."""
        d = VecPos(dest.x, dest.y)
        robot_pos = self.world.robot_pos
        angle = self.world.field_angle
        dangle = abs_angle_diff(dest.anglez - angle)

        diff_x = d.x - robot_pos.x
        diff_y = d.y - robot_pos.y

        # robot should face the ball
        if not self.world.enable_kick:

            y = self.world.ball_field.y
            DEST_REGION = self.cfg.DEST_REGION

            if y > 0:
                if abs(diff_x) < DEST_REGION and \
                        -DEST_REGION < diff_y < DEST_REGION / 3 and \
                        abs(dangle) < self.cfg.DEST_RE_ANGLE:
                    return True
                else:
                    return False
            else:
                if abs(diff_x) < DEST_REGION and \
                        -DEST_REGION / 3 < diff_y < DEST_REGION and \
                        abs(dangle) < self.cfg.DEST_RE_ANGLE:
                    return True

        elif self.world.enable_kick:
            if abs(diff_x) < 5 and abs(diff_y) < 5 and abs(dangle) < 5:
                return True

            elif self.near_goal():
                if abs(diff_x) < 5 and abs(diff_y) < 5 and abs(dangle) < 5:
                    return True
                else:
                    return False
        else:
            return False

    def near_goal(self):
        """Check if near goal."""
        if self.inside_goal():
            return True

        ENEMY_GOAL = get_attack_goal(self.cfg.GOAL_SHIFT)
        angle = self.world.field_angle
        if attacking_right() and abs(angle) > 80:
            return False
        elif attacking_left() and abs(angle) < 110:
            return False

        if get_dis(ENEMY_GOAL, self.world.robot_pos) < 150:
            return True
        else:
            return False

    def inside_goal(self):
        """Check if inside goal."""
        robot_pos = self.world.robot_pos
        angle = self.world.field_angle
        if attacking_left() and robot_pos.x < -400 and \
                abs(robot_pos.y) < 90 and abs(angle) > 90:
            return True
        elif attacking_right() and robot_pos.x > 400 and \
                abs(robot_pos.y) < 90 and abs(angle) < 90:
            return True


ApproachBall = parallel(selector(_Approach,
                                 WalkBehindBall),
                        TrackBall)
