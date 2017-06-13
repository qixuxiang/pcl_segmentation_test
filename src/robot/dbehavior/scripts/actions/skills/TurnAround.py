"""TurnAround skill."""

import rospy
from ...misc.bt import Action
from ...misc.utils.mathutil import get_angle


class TurnAround(Action):
    """TurnAround."""

    def init(self):
        """Init."""
        self.turned = 0

    def reset_(self):
        """Reset."""
        self.turned = 0

    def tick(self):
        """Tick."""
        rospy.logdebug('TurnAround tick')
        if self.world.see_ball:
            self.step()
            return self.success()

        else:
            # better use fieldangle
            if abs(self.turned) > 180:
                self.world.lost_ball.leftshift(10)
                return self.failure()
            else:
                # print self.turned
                self.turned += self.world.deltaData.z
                angle = get_angle(self.world.last_seen_ball_field)

                if angle > 0:
                    self.turn(20)
                else:
                    self.turn(-20)

                self.lookat(0, 58)
                return self.running()
