"""Goalie role."""
from geometry_msgs.msg import Vector3
from .Role import Role
from .Striker import Striker, sc
from ..headskills.ScanField import ScanField
from misc.bt import Action, ev_sequence
from misc.types.constant import ROLE_DEFENDER, ROLE_GOALIE, X, Y
from misc.types.field_geometry import attacking_left
from misc.utils.mathutil import get_magnitude


class _Striker(Role):
    def tick(self):
        if not self.world.lower_board_connected:
            self.gc.secsSinceUnpenalised.restart()

        if self.world.reentry and not self.world.see_ball:
            sc.tick()
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

        else:
            return self.success()

        # elif not self.team.closet_to_ball() and \
        #     get_magnitude(self.world.ball_field) < 150:
        #
        #     self.gaze_ball()
        #     self.face(self.world.ball_field)
        #     self.crouch()
        #     print '_Striker running 3'
        #     return self.running()


class _Goalie(Role):
    def init(self):
        self.scanfield = ScanField()
        self.striker = Striker()
        self.attack = False

    def tick(self):
        if not self.world.lower_board_connected:
            self.attack = False

        if self.attack:
            return self.success()

        if self.world.see_ball:
            if get_magnitude(self.world.ball_field) < 100:
                self.attack = True
                return self.success()
            else:
                self.face(self.world.ball_field)
                self.gaze_ball()
                return self.running()

        else:
            self.team.goalieAttacking = False
            self.crouch()
            self.scanfield.tick()
            print 'Goalie ticking 4'

        return self.running()


Goalie = ev_sequence(_Striker,
                     _Goalie,
                     Striker)
