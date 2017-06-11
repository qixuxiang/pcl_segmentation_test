# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:57:57+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: vision_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:37+08:00
# @Copyright: ZJUDancer

from .status_bb import StatusBlackBoard
from std_msgs.msg import String


class VisionBlackBoard(StatusBlackBoard):
    """BlackBoard for robot vision status"""
    def __init__(self):
        super(VisionBlackBoard, self).__init__()

        # subscribe
        self.subscribe('dvision', 'ball', String, self)
