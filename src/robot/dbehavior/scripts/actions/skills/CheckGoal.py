"""CheckGoal skill."""

from ...misc.bt import Action, parallel
from ..headskills.SearchGoal import SearchGoal


class _CheckGoal(Action):
    """CheckGoal."""

    def tick(self):
        self.crouch()


CheckGoal = parallel(_CheckGoal,
                     SearchGoal)
