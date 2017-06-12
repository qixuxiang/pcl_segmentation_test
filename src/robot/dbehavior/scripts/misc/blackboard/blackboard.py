"""BlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:40:48+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: blackboard.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:19+08:00
# @Copyright: ZJUDancer

from .motion_bb import MotionBlackBoard
from .vision_bb import VisionBlackBoard
from .params_bb import ParamsBlackBoard
from .gc_bb import GameControllerBlackBoard;


class BlackBoard(object):
    """BlackBoard for subscribing and publishing robot status."""

    def __init__(self):
        """Init BlackBoard."""
        super(BlackBoard, self).__init__()
        self.gc = GameControllerBlackBoard()
        self.motion = MotionBlackBoard()
        self.vision = VisionBlackBoard()
        # TODO(corenel) get robot id from GameController
        self.params = ParamsBlackBoard(0)
