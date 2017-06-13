"""GCPlay skill."""

import rospy
from .StaticStriker import StaticStriker
from ..game.Initial import Initial
from ..game.Ready import Ready
from ..game.Play import Play
from ..game.Finish import Finish
from ..game.Penalised import Penalised
from ...misc.bt import Action, condition, ev_selector, ev_sequence, parallel
from ...misc.types.constant import (STATE_INITIAL, STATE_READY, STATE_SET,
                                    STATE_PLAYING, STATE_FINISHED,
                                    STATE_INVALID)


@condition
def is_invalid(self):
    """Check if is invalid."""
    invalid = self.gc.state is STATE_INVALID
    return invalid


@condition
def is_initial(self):
    """Check if is initial."""
    initial = self.gc.state is STATE_INITIAL
    return initial


@condition
def is_ready(self):
    """Check if is ready."""
    ready = self.gc.state is STATE_READY
    return ready


@condition
def is_set(self):
    """Check if is set."""
    return self.gc.state is STATE_SET


@condition
def is_playing(self):
    """Check if is playing."""
    if self.gc.state is STATE_PLAYING and not self.gc.penalised:
        return True
    else:
        return False


@condition
def is_finished(self):
    """Check if is finished."""
    return self.gc.state is STATE_FINISHED


@condition
def is_penalised(self):
    """Check if is prnalised."""
    return self.gc.penalised or not self.world.lower_board_connected


class GC_Connected(Action):
    """GC connected."""

    def tick(self):
        """Tick."""
        if not self.gc.connected:
            rospy.logerr('[GCPlay] WARNING!GAME CONTROLLER is not connected')
        return self.success()


_GCPlay = ev_selector(ev_sequence(is_initial, Ready),
                      ev_sequence(is_ready, Ready),
                      ev_sequence(is_set, Ready),
                      ev_sequence(is_penalised, Penalised),
                      ev_sequence(is_playing, Play),
                      ev_sequence(is_finished, Finish),
                      ev_sequence(is_invalid, StaticStriker))

GCPlay = parallel(GC_Connected, _GCPlay)


# must ensure we can hear gamecontroller
