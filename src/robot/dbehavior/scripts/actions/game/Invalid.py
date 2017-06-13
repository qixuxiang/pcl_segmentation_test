"""Invalid state."""

from ...misc.bt import Action
from ...misc.bt import sequence
from ...misc.types.constant import ROLE_DEFENDER
# from roles.Striker import Striker
from headskill.ScanField import ScanField
from ...misc.utils.mathutil import get_dis, abs_angle_diff

sc = ScanField()


class _Invalid(Action):
    def init(self):
        self.got = False

    def tick(self):
        if not self.world.see_ball and self.world.ball_lost_sec() > 10:
            dest = self.team.get_start_pos(ROLE_DEFENDER)
            sc.tick()
            if not self.got_dest(dest) and not self.got:
                self.goto(dest)
                return self.running()
            else:
                self.got = True
                self.crouch()
                return self.running()
        else:
            self.got = False
            return self.success()

    def got_dest(self, pos):
        dis = get_dis(pos, self.world.robot_pos)
        diff_angle = abs_angle_diff(pos.anglez - self.world.robot_pos.anglez)
        # print dis, diff_angle

        if dis < 100 and diff_angle < 5:
            return True
        else:
            return False


Invalid = sequence(_Invalid)
