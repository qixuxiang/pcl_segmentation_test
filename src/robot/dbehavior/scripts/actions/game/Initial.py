"""Initial state."""

import rospy
from misc.bt import Action
from misc.bt import parallel
from misc.types.constant import (ROLE_NONE, ROLE_GOALIE, ROLE_DEFENDER,
                                    ROLE_MIDFIELDER, ROLE_STRIKER, GOALIE_ID,
                                    LEFT, RIGHT, STATE_INITIAL)
from misc.types.field_geometry import (LEFT_START_POS, RIGHT_START_POS,
                                          LEFT_START_POS_UP,
                                          RIGHT_START_POS_UP,
                                          MID_START_POS, MID_START_POS_UP)


# Actions within STATE_INITIAL, localize
class _Initial(Action):
    def tick(self):
        self.stand()
        self.lookat(0, 10)

        # if self.gc.state is STATE_INITIAL:
        #     self.set_initial_position()

        # if self.team.is_none():
        #     self.assign_role()

        return self.success()

    def set_initial_position(self):
        kick_direction = self.gc.kick_direction
        if -125 < self.world.field_angle < -55:
            if kick_direction is LEFT:
                print 'Initial: set position to RIGHT_START_POS_UP'
                self.set_position(RIGHT_START_POS_UP)
            else:
                print 'Initial: set position to LEFT_START_POS_UP'
                self.set_position(LEFT_START_POS_UP)

        elif 55 < self.world.field_angle < 125:
            if kick_direction is LEFT:
                self.set_position(RIGHT_START_POS)
                print 'Initial: set position to RIGHT_START_POS_UP'
            else:
                self.set_position(LEFT_START_POS)
                print 'Initial: set position to LEFT_START_POS_UP'

        elif abs(self.world.field_angle) < 45 or \
                abs(self.world.field_angle) > 135:
            self.set_position(MID_START_POS)
            print 'Initial: set position to MID_START_POS'

    def assign_role(self):
        print 'assign role!'
        if self.gc.player_id in GOALIE_ID:
            self.team.current_role = ROLE_GOALIE
        else:

            if not self.team.has_striker():
                self.team.be_striker()

            elif not self.team.has_defender():
                self.team.be_defender()

            elif not self.team.has_midfielder():
                self.team.be_midfielder()

            else:
                rospy.logerr('No role to assign !!!!!')

        rospy.loginfo('Assigned to {}'.format(self.team.get_role_str()))


Initial = parallel(_Initial)
