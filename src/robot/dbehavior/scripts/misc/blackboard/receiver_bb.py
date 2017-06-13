"""ReceiverBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-13T17:16:23+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: rev_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-13T17:16:44+08:00
# @Copyright: ZJUDancer


from .status_bb import StatusBlackBoard
from ..types.constant import ROBOTS_PER_TEAM


class ReceiverBlackBoard(StatusBlackBoard):
    """BlackBoard for robot receiver status."""

    def __init__(self):
        """Init ReceiverBlackBoard."""
        super(ReceiverBlackBoard, self).__init__()
        self.data = [None] * ROBOTS_PER_TEAM
        self.last_received = [0] * ROBOTS_PER_TEAM
        self.incapacitated = [True] * ROBOTS_PER_TEAM
