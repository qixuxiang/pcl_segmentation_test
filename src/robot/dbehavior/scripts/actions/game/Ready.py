"""Ready state."""
from geometry_msgs.msg import Vector3
from ..headskill.ScanField import ScanField
from ...misc.bt import Action, parallel
from ...misc.utils.mathutil import get_dis, abs_angle_diff
from ...misc.types.constant import GOALIE, KICKOFF, DEFENDER
from ...misc.types.constant import LEFT, RIGHT
from ...misc.types.field_geometry import (LEFT_GOAL_START_DOWN,
                                          LEFT_GOAL_START_UP,
                                          RIGHT_GOAL_START_UP,
                                          RIGHT_GOAL_START_DOWN,
                                          GOALIE_START_LEFT,
                                          GOALIE_START_RIGHT, attacking_left,
                                          DEFENDER_START_RIGHT,
                                          DEFENDER_START_LEFT)

sc = ScanField()


class _Ready(Action):
    def init(self):
        self.got = False

    def tick(self):
        # fixme, ensure i have a role.
        try:
            self.lookat(0, 15)
            self.enable_localization()
            dest = self.team.get_start_pos(self.team.currentRole)

            if not self.got_dest(dest) and not self.got:
                self.goto(dest)
                return self.running()
            else:
                self.got = True

                if not self.world.see_ball:
                    sc.tick()
                else:
                    self.gaze_ball()

                self.crouch()
            return self.success()

        except:
            print 'Ready failure: {}'.format(self.team.get_role_str())
            return self.success()

    def got_dest(self, pos):
        dis = get_dis(pos, self.world.robot_pos)
        diff_angle = abs_angle_diff(pos.anglez - self.world.robot_pos.anglez)

        if dis < 10 and diff_angle < 5:
            return True
        else:
            return False


class _Ready_manual(Action):
    def tick(self):
        pos = self.cfg.MANUAL_SET_POSITION
        kick_direction = self.gc.kick_direction

        self.crouch()
        self.lookat(0, 10)

        if self.team.id in KICKOFF and self.gc.kickoff:
            if attacking_left():
                self.set_position(Vector3(75, 0, -90))
            else:
                self.set_position(Vector3(-75, 0, 0))

        elif self.team.id in GOALIE:
            if kick_direction is LEFT:
                self.set_position(GOALIE_START_RIGHT)
            else:
                self.set_position(GOALIE_START_LEFT)

        elif self.team.id in DEFENDER:
            if kick_direction is LEFT:
                self.set_position(DEFENDER_START_RIGHT)
            else:
                self.set_position(DEFENDER_START_LEFT)

        else:
            if pos is 'up':
                if kick_direction is LEFT:
                    self.set_position(RIGHT_GOAL_START_UP)
                else:
                    self.set_position(LEFT_GOAL_START_UP)

            else:
                if kick_direction is LEFT:
                    self.set_position(RIGHT_GOAL_START_DOWN)
                else:
                    self.set_position(LEFT_GOAL_START_DOWN)

        return self.success()


Ready = _Ready_manual
