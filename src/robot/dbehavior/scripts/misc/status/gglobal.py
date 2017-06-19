"""Get global variables."""

from .world import World
from .team import Team
from ..blackboard.gc_bb import get_gc
from ..types.behaviour_request import BehaviourRequest

_blackboard = None
_world = None
_team = None
_b_req = BehaviourRequest()


def update_bb(bb):
    """Update global variables with BlackBoard."""
    global _blackboard
    global _world
    global _team
    _gc = get_gc()
    _blackboard = bb

    if not _world:
        _world = World(_blackboard)
    _world.update()

    if not _team:
        _team = Team(_blackboard, _world, _b_req)
    _team.update()

    _gc.update()


def get_team():
    """Get Team."""
    global _team
    return _team


def get_world():
    """Get world."""
    global _world
    return _world


def get_bb():
    """Get BlackBoard."""
    global _blackboard
    return _blackboard


def get_req():
    """Get BehaviourRequest."""
    global _b_req
    return _b_req


def init_req():
    """Init BehaviourRequest."""
    global _b_req
    global _world
    _b_req.reinit()
    _b_req.behaviour.lower_board_connected = _world.lower_board_connected
