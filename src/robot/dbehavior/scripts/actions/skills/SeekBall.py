"""SeekBall skill."""

from .TurnAround import TurnAround
from ..headskills.FindBall import FindBall
from ...misc.bt import selector


SeekBall = selector(FindBall,
                    TurnAround)
