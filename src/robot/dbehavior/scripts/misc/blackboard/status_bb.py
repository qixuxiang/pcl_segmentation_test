"""BlackBoard Prototype."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-11T14:33:49+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: status_bb.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-11T15:08:29+08:00
# @Copyright: ZJUDancer

import rospy
from ..types.vec_pos import make_vecpos_from_list


class StatusBlackBoard(object):
    """BlackBoard Prototype for subscribing and publishing robot status."""

    def __init__(self):
        """Init StatusBlackBoard."""
        super(StatusBlackBoard, self).__init__()

    def subscribe(self, topic, type, parent, var_name):
        """Subscribe to a topic in ros and callback to update its value."""
        rospy.Subscriber("{}".format(topic), type,
                         lambda msg: self.callback(parent, var_name,
                                                   topic, msg))

    def callback(self, parent, var_name, topic, msg):
        """Callback for ros subscription."""
        setattr(parent, var_name, msg.data)
        rospy.loginfo("{}: {}".format(topic, msg.data))

    def gparam(self, param_name, var_name, parent):
        """Get parameters from parameters server and save it to variables."""
        setattr(parent, var_name, rospy.get_param(param_name))

    def gparam_vecpos(self, param_name, var_name, parent):
        """Get VecPos from parameters server and save it to variables."""
        setattr(parent, var_name,
                make_vecpos_from_list(rospy.get_param(param_name)))