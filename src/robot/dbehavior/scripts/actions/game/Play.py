"""Play state."""

from .RoleChange import RoleChange
from ..roles.Striker import Striker
from ..roles.MidFielder import MidFielder
from ..roles.Goalie import Goalie
from ..roles.Defender import Defender
from ..skills.ReEntry import ReEntry
from misc.bt import Action, condition, ev_sequence, ev_selector
from misc.types.constant import (ROLE_GOALIE, ROLE_STRIKER, ROLE_DEFENDER,
                                    ROLE_MIDFIELDER, GOALIE, STRIKER, DEFENDER)


@condition
def is_striker(self):
    return self.team.id in STRIKER
    # return self.team.current_role is ROLE_STRIKER


@condition
def is_defender(self):
    return self.team.id in DEFENDER


@condition
def is_midfielder(self):
    return self.team.current_role is ROLE_MIDFIELDER


@condition
def is_goalie(self):
    return self.team.id in GOALIE
    # return self.team.current_role is ROLE_GOALIE


Play = ev_sequence(  # RoleChange,
        ReEntry,
        ev_selector(ev_sequence(is_striker, Striker),
                    ev_sequence(is_goalie, Goalie),
                    ev_sequence(is_defender, Defender)))

# ev_sequence(is_midfielder, MidFielder),
# ev_sequence(is_defender, Defender),
# ev_sequence(is_striker, Striker),
# ev_sequence(is_goalie, Goalie))

# ev_selector(ev_sequence(is_striker, Striker),
#             ev_sequence(is_defender, Defender),
#             ev_sequence(is_midfielder, MidFielder),
#             ev_sequence(is_goalie, Goalie)))
