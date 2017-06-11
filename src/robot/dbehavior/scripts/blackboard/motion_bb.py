# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T13:58:08+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: motion_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:09:02+08:00
# @Copyright: ZJUDancer

from .status_bb import StatusBlackBoard


class MotionBlackBoard(StatusBlackBoard):
    """BlackBoard for robot motion status"""
    def __init__(self):
        super(MotionBlackBoard, self).__init__()
