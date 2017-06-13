"""Attack skill."""

from .ApproachBall import ApproachBall
from .Dribble import Dribble
from .Kick import Kick
from ..headskills.SearchGoal import SearchGoal
from ..headskill.ScanField import ScanField
from ...misc.bt import Action, sequence, condition


class not_my_kickoff(Action):
    """Not my kickoff."""

    def tick(self):
        """Tick."""
        if self.world.see_ball and not self.gc.kickoff and \
                self.gc.secondaryTime > 2 and \
                (abs(self.world.ball_global.x) < 75 or
                 (abs(self.world.ball_global.y) < 75)):
            self.gaze_ball()
            self.crouch()
            return self.running()
        else:
            return self.success()


@condition
def is_goalie_attacking(self):
    """Check if is goalie attacking."""
    return self.team.is_goalie_attacking()


class wait_goalie(Action):
    """Wait goalie."""

    def init(self):
        """Init."""
        self.sc = ScanField()

    def tick(self):
        """Tick."""
        if self.team.is_goalie_attacking():
            if self.world.see_ball:
                self.crouch()
                self.gaze_ball()
            else:
                self.crouch()
                self.sc.tick()

            return self.running()
        else:
            return self.success()


Attack = sequence(not_my_kickoff,
                  wait_goalie,
                  ApproachBall,

                  # SearchGoal,
                  # Dribble,

                  Kick)
