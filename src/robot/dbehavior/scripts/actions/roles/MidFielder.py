"""MidFielder role."""
from .Role import Role
from ..skills.AvoidBall import AvoidBall
from ..headskills.ScanField import ScanField
from misc.types.constant import ROLE_MIDFIELDER
from misc.bt import sequence

sc = ScanField()


class _MidFielder(Role):
    def tick(self):
        print 'MidFielder ticking'
        if not self.world.see_ball and self.world.ball_lost_sec() > 10 and \
                self.away_from_start_pos(
                    self.team.get_start_pos(ROLE_MIDFIELDER)):
            # print 'go to start pos'
            self.enable_localization()
            self.go_to_start_pos(self.team.get_start_pos(ROLE_MIDFIELDER))
            sc.tick()
            return self.running()

        elif self.world.see_ball:
            return self.success()
        else:
            self.crouch()
            # print 'scan field'
            sc.tick()
            # self.enable_localization()
            return self.running()


# if at start point, then scanfield to search ball
# if see ball then go to avoid ball position
# if away from start point and lost ball for very long time then goback
MidFielder = sequence(_MidFielder, AvoidBall)
