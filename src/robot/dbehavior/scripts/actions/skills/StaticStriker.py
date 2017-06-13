"""StaticStriker skill."""

from geometry_msgs.msg import Vector3
from ..headskills.ScanField import ScanField
from ..roles.Striker import Striker
from ...misc.bt import Action, sequence, ev_sequence
from ...misc.types.field_geometry import attacking_left
from ...misc.utils.mathutil import get_magnitude

sc = ScanField()


class _StaticStriker(Action):
    def tick(self):
        x = self.world.robot_pos.x
        if attacking_left():
            if x > 300:
                if self.world.see_ball and self.world.ball_global.x < 200:
                    self.gaze_ball()
                    self.face(self.world.ball_field)
                    return self.running()

                elif self.world.see_ball and self.world.ball_field.x > 200:
                    return self.success()

                elif not self.world.see_ball and \
                        abs(self.world.field_angle) < 120:
                    self.turn(15)
                    sc.tick()
                    return self.running()
                else:
                    sc.tick()
                    self.crouch()
                    return self.running()
            else:
                if self.world.ball_lost_sec() > 120:
                    self.goto(Vector3(330, 100, -160))
                    sc.tick()
                    return self.running()
                else:
                    if self.world.see_ball:
                        self.gaze_ball()
                    else:
                        sc.tick()

                    self.crouch()
                    return self.running()
        else:
            if x < -300:
                if self.world.see_ball and self.world.ball_global.x > -200:
                    self.gaze_ball()
                    self.face(self.world.ball_field)
                    return self.running()

                elif self.world.see_ball and self.world.ball_field.x < -200:
                    return self.success()

                elif not self.world.see_ball and \
                        abs(self.world.field_angle) > 60:
                    self.turn(15)
                    sc.tick()
                    return self.running()
                else:

                    if self.world.see_ball:
                        self.gaze_ball()
                    else:
                        sc.tick()

                    self.crouch()
                    return self.running()
            else:
                if self.world.ball_lost_sec() > 120:
                    self.goto(Vector3(-330, -100, 20))
                    sc.tick()
                    return self.running()
                else:
                    if self.world.see_ball:
                        self.gaze_ball()
                    else:
                        sc.tick()

                    self.crouch()
                    return self.running()


StaticStriker = ev_sequence(_StaticStriker, Striker)
