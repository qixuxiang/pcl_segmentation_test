"""GameControllerBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T12:41:45+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: gc_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T12:41:50+08:00
# @Copyright: ZJUDancer

from std_msgs.msg import String
from .status_bb import StatusBlackBoard
from ..types.constant import (STATE_INVALID, STATE_PENALISED, INVALID, LEFT,
                              RIGHT, HALF_TIME, STATE_PLAYING, STATE_READY)
from ..utils.timer import Timer


class GameControllerBlackBoard(StatusBlackBoard):
    """BlackBoard for game controller status."""

    def __init__(self):
        """Init GameControllerBlackBoard."""
        super(GameControllerBlackBoard, self).__init__()
        self.cycle = 0
        self.connect = False
        self.connected = False

        self.firstHalf = True
        self.state = STATE_INVALID
        self.our_score = 0
        self.enemy_score = 0
        self.penalised = False
        self.secsRemaining = 600
        self.secsSinceGameStart = 0
        self.secondaryTime = 0
        self.secsTillUnpenalised = 0
        self.kickoff = False
        self.player_id = -1
        self.kick_direction = INVALID
        self.prev_state = INVALID
        self.secsSincePlay = None
        self.secsSinceUnpenalised = None

        # subscribe
        self.subscribe("/humanoid/game_controller/connected", String,
                       self, "connected")


_gc = GameControllerBlackBoard()


def get_gc():
    """Get GameControllerBlackBoard."""
    global _gc
    return _gc
