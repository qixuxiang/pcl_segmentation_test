"""Get global variables."""

# from Constant import UNKNOWN
# from util.actioncommand import crouch, head
# from World import World
# from Team import Team

_blackboard = None
_world = None
_team = None
# _b_req = BehaviourRequest()


def update_bb(bb):
    """Update BlackBoard."""
    global _blackboard
    global _world
    global _team
    _blackboard = bb

    # if not _world:
    #     _world = World()
    # _world.update(_blackboard)

    # if not _team:
    #     _team = Team(_world, _b_req)
    # _team.update(_blackboard)


# def get_team():
#     global _team
#     return _team


# def get_world():
#     """Get world."""
#     global _world
#     return _world


def get_bb():
    """Get BlackBoard."""
    global _blackboard
    return _blackboard


# def get_req():
#     global _b_req
#     return _b_req


# after this function get called, all old ref to _b_req is not changed, why?
# def init_req():
#     global _b_req
#     # _b_req.actions.body = walk(0, 0, 0)
#     _b_req.update_gait_vec = False
#     _b_req.actions.body = crouch()
#     _b_req.actions.head = head(0, 70)
#     _b_req.enable_localization = False
#     _b_req.destination = robot.RobotState(UNKNOWN, UNKNOWN, 0)
#     _b_req.kick = False
#     _b_req.saveimage = False
#     _b_req.resetLocalization = False
#     _b_req.reset_point = robot.RobotState(UNKNOWN, UNKNOWN, 0)
