"""TrackBall headskill."""

from misc.bt import Action


class TrackBall(Action):
    """TrackBall."""

    def tick(self):
        """Tick."""
        if self.bb.vision.see_circle:  # or self.world.mem_ball_valid:
            self.gaze_at_fuck()
            return self.success()
        else:
            self.lookat(0, 15)
            return self.failure()


# an instance of TrackBall, just for test usage
trackball = TrackBall()
