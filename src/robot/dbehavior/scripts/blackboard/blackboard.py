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


class BlackBoard(object):
    """BlackBoard for subscribing and publishing robot status"""
    def __init__(self):
        super(BlackBoard, self).__init__()
        self.motion = MotionBlackBoard()
        self.vision = VisionBlackBoard()
