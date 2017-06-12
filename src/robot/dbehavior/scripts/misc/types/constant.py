"""Constant."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T13:04:50+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: constant.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T13:27:43+08:00
# @Copyright: ZJUDancer

import rospy
from ..types.vec_pos import VecPos


HALF_TIME = rospy.get_param("/dbehaviour/constant/HALF_TIME")
BALL_DROP_CYCLE = rospy.get_param("/dbehaviour/constant/BALL_DROP_CYCLE")
LOST_BALL_DELAY = rospy.get_param("/dbehaviour/constant/LOST_BALL_DELAY")
MAX_VIEW_DIST = rospy.get_param("/dbehaviour/constant/MAX_VIEW_DIST")
UNKNOWN = rospy.get_param("/dbehaviour/constant/UNKNOWN")
MAX_PLAT_YAW = rospy.get_param("/dbehaviour/constant/MAX_PLAT_YAW")
MAX_PLAT_PITCH = rospy.get_param("/dbehaviour/constant/MAX_PLAT_PITCH")
MIN_PLAT_PITCH = rospy.get_param("/dbehaviour/constant/MIN_PLAT_PITCH")
# game controller state
STATE_INITIAL = rospy.get_param("/dbehaviour/constant/STATE_INITIAL")
STATE_READY = rospy.get_param("/dbehaviour/constant/STATE_READY")
STATE_SET = rospy.get_param("/dbehaviour/constant/STATE_SET")
STATE_PLAYING = rospy.get_param("/dbehaviour/constant/STATE_PLAYING")
STATE_FINISHED = rospy.get_param("/dbehaviour/constant/STATE_FINISHED")
STATE_INVALID = rospy.get_param("/dbehaviour/constant/STATE_INVALID")
STATE_PENALISED = rospy.get_param("/dbehaviour/constant/STATE_PENALISED")
ROBOTS_PER_TEAM = rospy.get_param("/dbehaviour/constant/ROBOTS_PER_TEAM")
TEAM_BLUE = rospy.get_param("/dbehaviour/constant/TEAM_BLUE")
TEAM_CYAN = rospy.get_param("/dbehaviour/constant/TEAM_CYAN")
TEAM_RED = rospy.get_param("/dbehaviour/constant/TEAM_RED")
TEAM_MAGENTA = rospy.get_param("/dbehaviour/constant/TEAM_MAGENTA")
# Stable
STABLE = rospy.get_param("/dbehaviour/constant/STABLE")
RIGHTDOWN = rospy.get_param("/dbehaviour/constant/RIGHTDOWN")
LEFTDOWN = rospy.get_param("/dbehaviour/constant/LEFTDOWN")
FRONTDOWN = rospy.get_param("/dbehaviour/constant/FRONTDOWN")
BACKDOWN = rospy.get_param("/dbehaviour/constant/BACKDOWN")
# speed
WALK_SPEED = rospy.get_param("/dbehaviour/constant/WALK_SPEED")
TURN_SPEED = rospy.get_param("/dbehaviour/constant/TURN_SPEED")
# Role enums
ROLE_NONE = rospy.get_param("/dbehaviour/constant/ROLE_NONE")
ROLE_GOALIE = rospy.get_param("/dbehaviour/constant/ROLE_GOALIE")
ROLE_STRIKER = rospy.get_param("/dbehaviour/constant/ROLE_STRIKER")
ROLE_DEFENDER = rospy.get_param("/dbehaviour/constant/ROLE_DEFENDER")
ROLE_MIDFIELDER = rospy.get_param("/dbehaviour/constant/ROLE_MIDFIELDER")
# image
IMAGE_WIDTH = rospy.get_param("/dbehaviour/constant/IMAGE_WIDTH")
IMAGE_HEIGHT = rospy.get_param("/dbehaviour/constant/IMAGE_HEIGHT")
VIEW_CENTER = VecPos(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2)
# Attack side
INVALID = rospy.get_param("/dbehaviour/constant/INVALID")
LEFT = rospy.get_param("/dbehaviour/constant/LEFT")
RIGHT = rospy.get_param("/dbehaviour/constant/RIGHT")
# ball
NUM_LOST_FRAMES_TEAM_BALL_LOST = \
    rospy.get_param("/dbehaviour/constant/NUM_LOST_FRAMES_TEAM_BALL_LOST")
NUM_LOST_FRAMES_CAN_SEE_BALL = \
    rospy.get_param("/dbehaviour/constant/NUM_LOST_FRAMES_CAN_SEE_BALL")
# ATTACK MODE
ONLY_KICK = rospy.get_param("/dbehaviour/constant/ONLY_KICK")
KICK_NEAR_GOAL = rospy.get_param("/dbehaviour/constant/KICK_NEAR_GOAL")
# roles
GOALIE_ID = rospy.get_param("/dbehaviour/constant/GOALIE_ID")
GOALIE = rospy.get_param("/dbehaviour/constant/GOALIE")
STRIKER = rospy.get_param("/dbehaviour/constant/STRIKER")
DEFENDER = rospy.get_param("/dbehaviour/constant/DEFENDER")
KICKOFF = rospy.get_param("/dbehaviour/constant/KICKOFF")
# pos
X = rospy.get_param("/dbehaviour/constant/X")
Y = rospy.get_param("/dbehaviour/constant/Y")


def get_role_str(self, role):
    """Get role string."""
    if role is ROLE_NONE:
        return 'NONE'
    elif role is ROLE_GOALIE:
        return 'Goalie'
    elif role is ROLE_STRIKER:
        return 'Striker'
    elif role is ROLE_DEFENDER:
        return 'Defender'
    elif role is ROLE_MIDFIELDER:
        return 'MidFielder'
    else:
        return 'Role id invalid!'
