"""ConfigBlackBoard."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T15:03:34+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: config_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:52+08:00
# @Copyright: ZJUDancer

from .status_bb import StatusBlackBoard


class ConfigBlackBoard(StatusBlackBoard):
    """BlackBoard for robot configs."""

    def __init__(self):
        """Init ConfigBlackBoard."""
        super(ConfigBlackBoard, self).__init__()
