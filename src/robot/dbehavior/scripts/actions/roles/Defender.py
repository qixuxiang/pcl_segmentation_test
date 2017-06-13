"""Defender role."""
from geometry_msgs.msg import Vector3
from .Role import Role
from .Striker import Striker
from ..headskill.ScanField import ScanField
from ...misc.bt import sequence, ev_sequence
from ...misc.types.constant import ROLE_DEFENDER, X, Y
from ...misc.types.field_geometry import attacking_left
from ...misc.utils.mathutil import get_magnitude

sc = ScanField()


class midentry(Role):
    """MidEntry."""

    def tick(self):
        """Tick."""
        sc.tick()

        if not self.world.lower_board_connected:
            self.gc.secsSinceUnpenalised.restart()

        if self.world.reentry and not self.world.see_ball:
            if attacking_left():
                self.goto(Vector3(X, Y, 179))

                if self.got_dest(Vector3(X, Y, 179)):
                    self.world.reentry = False
            else:
                self.goto(Vector3(X, Y, 0))
                if self.got_dest(Vector3(X, Y, 0)):
                    self.world.reentry = False
            print '_Striker running 2'
            return self.running()

        elif not self.world.see_ball and \
                ((self.gc.secsSincePlay.elapsed() < 5000000 and
                  self.gc.connected) or
                 self.gc.secsSinceUnpenalised.elapsed() < 5000000):
            if attacking_left():

                self.goto(Vector3(300, 0, 179))
            else:

                self.goto(Vector3(-300, 0, 0))
            print '_Striker running 1'
            return self.running()

        # elif not self.team.closet_to_ball() and \
        #     get_magnitude(self.world.ball_field) < 150:
        #
        #     self.gaze_ball()
        #     self.face(self.world.ball_field)
        #     self.crouch()
        #     print '_Striker running 3'
        #     return self.running()

        else:
            return self.success()


class _Defender(Role):
    def init(self):
        self.attack = False

    def tick(self):
        if not self.world.lower_board_connected:
            self.attack = False

        if self.attack:
            return self.success()

        if not self.world.see_ball:
            sc.tick()
            self.crouch()
            return self.running()

        elif self.world.see_ball:

            # print 'ball x: {}'.format(self.world.ball_field)

            if abs(self.world.ball_field) < 200:
                self.attack = True
                return self.success()
            else:
                # self.face(self.world.ball_field)
                self.gaze_ball()
                return self.running()
        else:
            sc.tick()
            self.crouch()
            return self.running()


# TODO(mwx), test this
Defender = ev_sequence(midentry, _Defender, Striker)
