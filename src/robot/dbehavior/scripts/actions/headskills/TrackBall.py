"""TrackBall headskill."""

from ...misc.bt import Action


class TrackBall(Action):
    """TrackBall."""

    def tick(self):
        """Tick."""
        if self.world.see_ball:  # or self.world.mem_ball_valid:
            self.gaze_ball()
            return self.success()
        else:
            self.lookat(0, 15)
            return self.failure()


# an instance of TrackBall, just for test usage
trackball = TrackBall()
