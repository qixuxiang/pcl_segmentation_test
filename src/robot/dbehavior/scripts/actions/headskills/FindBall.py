"""FindBall headskill."""

from misc.bt import Action
from misc.types.vec_pos import VecPos
from misc.utils.timer import Timer
from misc.utils.mathutil import get_magnitude

GazePlats = [
    VecPos(0, 0),
    VecPos(60, 15),
    VecPos(0, 12),
    VecPos(-60, 15)
]


class FindBall(Action):
    """Find ball.

    when robot is not turning, scan field, else look down
    """

    def init(self):
        """Init."""
        self.timer = Timer()
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()

    def tick(self):
        """Tick."""
        self.enable_localization()
        if not self.world.see_ball and self.world.ball_lost_sec() < 10 and \
                get_magnitude(self.world.last_seen_ball_field) < 50:
            return self.failure()

        if self.gaze_ball():
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()

            return self.success()

        else:
            if self.timer.elapsed() > 1000000:
                self.timer.restart()
                if not self.next_plat():
                    return self.failure()

            self.lookat(self.curPlat.x, self.curPlat.y, 10, 10)
            return self.running()

    def next_plat(self):
        """Nect play."""
        try:
            self.curPlat = self.iter.next()
            self.world.scanning = True
            return True
        except StopIteration:
            # GazePlats.reverse()
            self.world.scanning = False
            self.iter = iter(GazePlats)
            self.curPlat = self.iter.next()
            return False
