"""RoleChange state."""
import rospy
from ...misc.bt import Action
from ...misc.types.constant import get_role_str, GOALIE_ID, GOALIE
from ...misc.utils.timer import Timer, get_current_time


class RoleChange(Action):
    """RoleChange."""

    def init(self):
        """Init."""
        self.last_change_time = Timer()

    def tick(self):
        """Tick."""
        prev_role = self.team.currentRole
        # Goalie don't change role
        if self.team.is_goalie():
            return self.success()
        else:
            if self.team.is_striker():
                return self.success()

            if self.last_change_time.elapsed() > 2000000:
                if self.team.is_none():
                    self.cover()

                elif self.team.is_striker():
                    if not self.team.closet_to_ball() and \
                            self.world.ball_lost_sec() > 5000000:
                        self.cover()

                    elif self.world.ball_lost_sec() > 60:
                        self.cover()

                elif self.team.is_midfiedler() or self.team.is_defender():
                    if self.team.closet_to_ball():
                        self.team.be_striker()
                    else:
                        self.cover()

        if prev_role is not self.team.currentRole:
            self.last_change_time.restart()
            rospy.loginfo('[RoleChange {}] {} --> {}'
                          .format(get_current_time(),
                                  get_role_str(prev_role),
                                  self.team.get_role_str()))

        return self.success()

    def cover(self):
        """Cover."""
        if self.gc.player_id in GOALIE:
            self.team.be_goalie()

        else:
            if not self.team.has_defender():
                self.team.be_defender()
            elif not self.team.has_striker() and not self.team.is_defender():
                self.team.be_striker()
            elif not self.team.has_midfielder() and \
                    not self.team.is_defender():
                self.team.be_midfielder()
            else:
                pass
                # self.team.be_defender()


# ensure there's always no more one defender / midfielder / striker
