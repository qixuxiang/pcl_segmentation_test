"""Role."""
from misc.bt import Action
from misc.utils.mathutil import get_dis

MAX_DIS_START_POS = 100


class Role(Action):
    """Role."""

    def tick(self):
        """Tick."""
        pass

    def away_from_start_pos(self, start_pos):
        """Away from start pos."""
        current_pos = self.world.robot_pos
        dis = get_dis(current_pos, start_pos)
        diff_angle = abs(start_pos.anglez - current_pos.anglez)

        if dis > MAX_DIS_START_POS or diff_angle > 10:
            return True
        else:
            return False

    def go_to_start_pos(self, start_pos):
        """Go to strat pos."""
        self.goto(start_pos)
