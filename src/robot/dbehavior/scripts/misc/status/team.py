"""Team."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T16:02:04+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: team.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T16:02:11+08:00
# @Copyright: ZJUDancer

from ..utils.mathutil import (get_angle, get_magnitude, get_dis,
                              angle_normalization)
from ..utils.calc_attack_point import get_rub
from ..types.constant import (WALK_SPEED, TURN_SPEED, ROBOTS_PER_TEAM,
                              NUM_LOST_FRAMES_TEAM_BALL_LOST, ROLE_NONE,
                              ROLE_DEFENDER, ROLE_GOALIE, ROLE_MIDFIELDER,
                              ROLE_STRIKER, get_role_str, LEFT)
from ..types.field_geometry import (STRIKER_POS_KICKOFF,
                                    STRIKER_POS_NONE_KICK_OFF,
                                    DEFENDER_POS, MIDFIELDER_POS, GOALIE_POS,
                                    inverse_global_pos_by_side)
from ..blackboard.gc_bb import get_gc


class Team(object):
    """Used for team status."""

    def __init__(self, blackboard, world, req):
        """Init."""
        self.bb = blackboard
        self.world = world
        self.req = req
        self.cycle = 0

        self.time_to_reach_ball = 9999
        self.last_time_to_reach_ball = 9999
        self.time_to_reach_defender = 9999
        self.time_to_reach_midfielder = 9999
        self.time_to_reach_striker = 9999
        self.current_role = ROLE_NONE
        self.goalie_attacking = False

        self.data = None
        self.incapacitated = None
        self.last_received = None

    def update(self):
        """Update."""
        gc = get_gc()
        self.id = gc.player_id

        self.cycle += 1
        self.data = self.bb.receiver.data
        self.last_received = self.bb.receiver.last_received
        self.incapacitated = self.bb.receiver.incapacitated

        self.update_times()

        self.req.team.time_to_reach_ball = self.time_to_reach_ball
        self.req.team.time_to_reach_striker = self.time_to_reach_striker
        self.req.team.time_to_reach_defender = self.time_to_reach_defender
        self.req.team.time_to_reach_midfielder = self.time_to_reach_midfielder

        if self.current_role is None:
            raise Exception('CurrentRole is None')
        else:
            self.req.team.current_role = self.current_role

        self.req.team.goalie_attacking = self.goalie_attacking

    def update_times(self):
        """Update time."""
        # 1. time to reach ball
        if not self.world.stable:
            self.time_to_reach_ball = 9999.0
            self.last_time_to_reach_ball = 9999.0

        if self.world.enable_checkgoal:
            self.time_to_reach_ball = self.last_time_to_reach_ball

        if self.world.see_ball:
            rub = get_rub()
            angle = get_angle(rub)
            dis = get_magnitude(rub)

            self.time_to_reach_ball = abs(angle / TURN_SPEED) + dis / WALK_SPEED

            if 0 < rub.x < 30 and -20 < rub.y < 20:
                self.time_to_reach_ball = 0

            self.last_time_to_reach_ball = self.time_to_reach_ball
        else:

            self.time_to_reach_ball = 9999.0

        # update time to reach role
        self.time_to_reach_striker = \
            calc_time_to_reach_pos(self.world.robot_pos,
                                   self.get_start_pos(ROLE_STRIKER))
        self.time_to_reach_midfielder = \
            calc_time_to_reach_pos(self.world.robot_pos,
                                   self.get_start_pos(ROLE_MIDFIELDER))
        self.time_to_reach_defender = \
            calc_time_to_reach_pos(self.world.robot_pos,
                                   self.get_start_pos(ROLE_DEFENDER))

    def get_start_pos(self, role):
        """Get start pos."""
        gc = get_gc()
        kick_direction = gc.kick_direction

        if gc.kickoff:
            striker_pos = STRIKER_POS_KICKOFF
        else:
            striker_pos = STRIKER_POS_NONE_KICK_OFF

        defender_pos = DEFENDER_POS
        midfielder_pos = MIDFIELDER_POS
        goalie_pos = GOALIE_POS

        if kick_direction is LEFT:
            # ? is this bug?
            striker_pos = inverse_global_pos_by_side(striker_pos)
            defender_pos = inverse_global_pos_by_side(DEFENDER_POS)
            midfielder_pos = inverse_global_pos_by_side(MIDFIELDER_POS)
            goalie_pos = inverse_global_pos_by_side(GOALIE_POS)

        if role is ROLE_STRIKER:
            return striker_pos
        elif role is ROLE_MIDFIELDER:
            return midfielder_pos
        elif role is ROLE_DEFENDER:
            return defender_pos
        elif role is ROLE_GOALIE:
            return goalie_pos

    def is_teammate_active(self, index):
        """Check if teammate is active."""
        return not self.incapacitated[index]

    def has_team_lost_ball(self):
        """Check if team has lost ball."""
        for i in xrange(ROBOTS_PER_TEAM):
            print self.data[i].ballLostCount
            if self.is_teammate_active(i) and \
                    self.data[i].ballLostCount < \
                    NUM_LOST_FRAMES_TEAM_BALL_LOST:
                return False

        return True

    def num_active_player(self):
        """Get number of active player."""
        count = 0
        for i in xrange(ROBOTS_PER_TEAM):
            if not self.incapacitated[i]:
                count += 1

        return count

    def get_time_to_reach_ball(self, index):
        """Get time to reach ball."""
        return self.data[index].behaviourSharedData.time_to_reach_ball

    def get_time_to_reach_striker(self, index):
        """Get time to reach Striker."""
        return self.data[index].behaviourSharedData.time_to_reach_striker

    def get_time_to_reach_midfielder(self, index):
        """Get time to reach MidFielder."""
        return self.data[index].behaviourSharedData.time_to_reach_midfielder

    def get_time_to_reach_defender(self, index):
        """Get time to reach Defender."""
        return self.data[index].behaviourSharedData.time_to_reach_defender

    def has_striker(self):
        """Check if Striker exists."""
        for i in xrange(ROBOTS_PER_TEAM):
            data = self.data[i]
            if data.behaviourSharedData.current_role is ROLE_STRIKER and \
                    self.is_teammate_active(i):
                return True
        return False

    def has_defender(self):
        """Check if Defender exists."""
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_DEFENDER and \
                    self.is_teammate_active(i):
                # print 'robot {} is defender'.format(i + 1)
                return True
                # else:
                # print 'no defender'
        return False

    def has_midfielder(self):
        """Check if MidFielder exists."""
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_MIDFIELDER and \
                    self.is_teammate_active(i):
                return True
        return False

    def has_goalie(self):
        """Check if Goalie exists."""
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_GOALIE and self.is_teammate_active(i):
                return True
        return False

    def closet_to_ball(self):
        """Check if self is closet to ball."""
        # return get_order_close_to_ball(1)

        if not self.world.see_ball:
            return False

        smallest_time = 10000
        index = -1
        for i in xrange(ROBOTS_PER_TEAM):
            t = self.get_time_to_reach_ball(i)
            if self.is_teammate_active(i) and t < smallest_time:
                index = i
                smallest_time = t

        if index == self.id - 1 and smallest_time < 9999:
            return True
        else:
            return False

    def get_order_close_to_ball(self, order):
        """Get order close to ball."""
        if not self.world.see_ball:
            return False
            # fixme, use sorted dict

    def closet_to_defender(self):
        """Check if self is closet to Defender."""
        smallest_time = 10000
        index = -1
        for i in xrange(ROBOTS_PER_TEAM):
            # if self.get_role(i) is ROLE_DEFENDER:
            #     if self.is_self(i):
            #         return True
            #     else:
            #         return False

            t = self.get_time_to_reach_defender(i)
            if self.is_teammate_active(i) and t < smallest_time:
                index = i
                smallest_time = t

        if self.is_self(index) and smallest_time < 9999:
            return True
        else:
            return False

    def closet_to_striker(self):
        """Check if self is closet to Striker."""
        smallest_time = 10000
        index = -1
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_STRIKER:
                if self.is_self(i):
                    return True
                else:
                    return False

            t = self.get_time_to_reach_striker(i)
            if self.is_teammate_active(i) and t < smallest_time:
                index = i
                smallest_time = t

        if self.is_self(index) and smallest_time < 9999:
            return True
        else:
            return False

    def closet_to_midfielder(self):
        """Check if self is closet to MidFielder."""
        smallest_time = 10000
        index = -1
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_MIDFIELDER:
                if self.is_self(i):
                    return True
                else:
                    return False

            t = self.get_time_to_reach_midfielder(i)
            if self.is_teammate_active(i) and t < smallest_time:
                index = i
                smallest_time = t

        if self.is_self(index) and smallest_time < 9999:
            return True
        else:
            return False

    def is_self(self, index):
        """Check if it's self."""
        return index == self.id - 1

    def goalie_incapacitated(self):
        """Check if Goalie is incapacitated."""
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_GOALIE and \
                    not self.is_teammate_active(i):
                return True
        return False

    def not_closet_to_current_role(self):
        """Check if self is not clost to current role."""
        if self.is_defender() and not self.closet_to_defender():
            return True
        elif self.is_midfiedler() and not self.closet_to_midfielder():
            return True
        elif self.is_striker() and not self.closet_to_striker():
            return True
        else:
            return False

    def be_striker(self):
        """To be a Striker."""
        self.current_role = ROLE_STRIKER

    def be_defender(self):
        """To be a Defender."""
        self.current_role = ROLE_DEFENDER

    def be_midfielder(self):
        """To be a MidFielder."""
        self.current_role = ROLE_MIDFIELDER

    def be_goalie(self):
        """To be a Goalie."""
        self.current_role = ROLE_GOALIE

    def is_striker(self):
        """Check if self is Striker."""
        return self.current_role is ROLE_STRIKER

    def is_defender(self):
        """Check if self is Defender."""
        return self.current_role is ROLE_DEFENDER

    def is_midfiedler(self):
        """Check if self is MidFielder."""
        return self.current_role is ROLE_MIDFIELDER

    def is_goalie(self):
        """Check if self is Goalie."""
        return self.current_role is ROLE_GOALIE

    def is_none(self):
        """Check if self is None."""
        return self.current_role is ROLE_NONE

    def teammate_striking(self):
        """Check if teammate is striking."""
        # todo, fixme
        return False

    def is_goalie_attacking(self):
        """Check if Goalie is attacking."""
        for i in xrange(ROBOTS_PER_TEAM):
            if self.get_role(i) is ROLE_GOALIE and \
                    not self.is_teammate_active(i) and \
                    self.get_goalieattacing(i):
                return True
        return False

    def get_role(self, index):
        """Get role by index."""
        return self.data[index].behaviourSharedData.current_role

    def get_role_str(self):
        """Get role string."""
        return get_role_str(self.current_role)

    def get_goalieattacing(self, index):
        """Get Goalie attacking."""
        return self.data[index].behaviourSharedData.goalie_attacking


# if see ball and very close but not the closet,
# then move away and go to defend position or
# another so i need to known about what place i need to go.7


def calc_time_to_reach_pos(robot_pos, dest_pos):
    """Calculate time to reach pos."""
    dis = get_dis(robot_pos, dest_pos)
    angle = abs(angle_normalization(robot_pos.z - dest_pos.z))
    return dis / WALK_SPEED + angle / TURN_SPEED / 2
