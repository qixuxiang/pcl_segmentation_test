"""GameControllerBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T12:41:45+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: gc_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T12:41:50+08:00
# @Copyright: ZJUDancer

from .status_bb import StatusBlackBoard


class GameControllerBlackBoard(StatusBlackBoard):
    """BlackBoard for game controller status."""

    def __init__(self):
        """Init GameControllerBlackBoard."""
        super(GameControllerBlackBoard, self).__init__()
