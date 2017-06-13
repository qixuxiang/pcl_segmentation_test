"""class FieldGeometry."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T17:17:42+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: field_geometry.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T17:17:46+08:00
# @Copyright: ZJUDancer

import rospy
from geometry_msgs.msg import Vector3
from math import atan2, degrees
from ..utils.mathutil import angle_normalization, degree_between
from ..types.vec_pos import VecPos
from ..types.constant import MAX_VIEW_DIST, LEFT
from ..blackboard.gc_bb import get_gc


# field model
FIELD_LENGTH = rospy.get_param("/dvision/field_model/field_length")
FIELD_WIDTH = rospy.get_param("/dvision/field_model/field_width")
GOAL_DEPTH = rospy.get_param("/dvision/field_model/goal_depth")
GOAL_WIDTH = rospy.get_param("/dvision/field_model/goal_width")
GOAL_HEIGHT = rospy.get_param("/dvision/field_model/goal_height")
GOAL_AREA_LENGTH = rospy.get_param("/dvision/field_model/goal_area_length")
GOAL_AREA_WIDTH = rospy.get_param("/dvision/field_model/goal_area_width")
PENALTY_MARK_DISTANCE = \
    rospy.get_param("/dvision/field_model/penalty_mark_distance")
CENTER_CIRCLE_DIAMETER = \
    rospy.get_param("/dvision/field_model/center_circle_diameter")
BORDER_STRIP_WIDTH = rospy.get_param("/dvision/field_model/border_strip_width")

HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0  # 450
HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0  # 300
HALF_GOAL_WIDTH = GOAL_WIDTH / 2.0  # 130
CENTER_CIRCLE_RADIUS = CENTER_CIRCLE_DIAMETER / 2.0  # 75
PENALTY_X = HALF_FIELD_LENGTH - PENALTY_MARK_DISTANCE  # 240

# goal pos
BLUE_GOAL_CENTER = VecPos(-HALF_FIELD_LENGTH, 0)
BLUE_GOAL_LEFT = VecPos(-HALF_FIELD_LENGTH, -HALF_FIELD_WIDTH)
BLUE_GOAL_RIGHT = VecPos(-HALF_FIELD_LENGTH, HALF_FIELD_WIDTH)

RED_GOAL_CENTER = VecPos(HALF_FIELD_LENGTH, 0)
RED_GOAL_LEFT = VecPos(HALF_FIELD_LENGTH, HALF_FIELD_WIDTH)
RED_GOAL_RIGHT = VecPos(HALF_FIELD_LENGTH, -HALF_FIELD_WIDTH)

# pickup pos
UP_PICKUP_ENTRY_POINT = Vector3(0, HALF_FIELD_WIDTH, -90)
DOWN_PICKUP_ENTRY_POINT = Vector3(0, -HALF_FIELD_WIDTH, 90)

# role pos
DEFENDER_POS = Vector3(-HALF_FIELD_LENGTH / 2, -HALF_FIELD_WIDTH / 2,
                       degrees(atan2(HALF_FIELD_WIDTH / 2,
                                     HALF_FIELD_LENGTH / 2)))
MIDFIELDER_POS = Vector3(-HALF_FIELD_LENGTH / 4, HALF_FIELD_WIDTH / 2,
                         -degrees(atan2(HALF_FIELD_WIDTH / 2,
                                        HALF_FIELD_LENGTH / 4)))
GOALIE_POS = Vector3(-HALF_FIELD_LENGTH + 50, 0, 0)
STRIKER_POS_KICKOFF = Vector3(-CENTER_CIRCLE_RADIUS, 0, 0)  # (-75, 0, 0)
STRIKER_POS_NONE_KICK_OFF = Vector3(-200, 0, 0)

# start pos
LEFT_START_POS = Vector3(-PENALTY_X, -FIELD_WIDTH, 90)
RIGHT_START_POS = Vector3(PENALTY_X, -FIELD_WIDTH, 90)

LEFT_START_POS_UP = Vector3(-PENALTY_X, FIELD_WIDTH, -90)
RIGHT_START_POS_UP = Vector3(PENALTY_X, FIELD_WIDTH, -90)

MID_START_POS = Vector3(0, -FIELD_WIDTH, 0)
MID_START_POS_UP = Vector3(0, FIELD_WIDTH, 0)

# attack targets
LEFT_ATTACK_POS_UP = VecPos(-HALF_FIELD_LENGTH, HALF_FIELD_WIDTH)
LEFT_ATTACK_POS_DOWN = VecPos(-HALF_FIELD_LENGTH, -HALF_FIELD_WIDTH)
RIGHT_ATTACK_POS_UP = VecPos(HALF_FIELD_LENGTH, HALF_FIELD_WIDTH)
RIGHT_ATTACK_POS_DOWN = VecPos(HALF_FIELD_LENGTH, -HALF_FIELD_WIDTH)

LEFT_GOAL_START_UP = Vector3(-350, 90, 0)
LEFT_GOAL_START_DOWN = Vector3(-350, -90, 0)

RIGHT_GOAL_START_UP = Vector3(350, 90, -90)
RIGHT_GOAL_START_DOWN = Vector3(350, -90, -90)

GOALIE_START_LEFT = Vector3(-450, 0, 0)
GOALIE_START_RIGHT = Vector3(450, 0, -90)

DEFENDER_START_LEFT = Vector3(-350, 0, 0)
DEFENDER_START_RIGHT = Vector3(350, 0, -90)

ATTACK_TARGET = BLUE_GOAL_CENTER.copy()


def get_attack_goal(kick_shift):
    """Get attack goal."""
    gc = get_gc()
    attack_side = gc.kick_direction

    if attack_side is LEFT:
        enemy_goal = BLUE_GOAL_CENTER.copy()
        enemy_goal.y -= kick_shift
    else:
        enemy_goal = RED_GOAL_CENTER.copy()
        enemy_goal.y += kick_shift

    return enemy_goal


def get_attack_target():
    """Get attack target."""
    global ATTACK_TARGET
    return ATTACK_TARGET


def set_attack_target(target):
    """Set attack target.

    only accecpt VecPos
    """
    global ATTACK_TARGET
    ATTACK_TARGET = target.copy()


def attacking_left():
    """Attacking left."""
    gc = get_gc()

    if gc.kick_direction is LEFT:
        return True
    else:
        return False


def facing_goal(robot_pos):
    """Facing goal."""
    left, right = attack_angle_range(robot_pos)
    diff = angle_normalization(left - right)

    left += diff / 4
    right -= diff / 4

    # print diff

    angle = robot_pos.anglez

    if angle_normalization(angle - left) < 0 and \
            angle_normalization(angle - right) > 0:
        return True
    else:
        return False


def attack_angle_range(robot_pos):
    """Attack angle range."""
    gc = get_gc()
    attack_side = gc.kick_direction

    if attack_side is LEFT:
        left = degree_between(robot_pos, BLUE_GOAL_LEFT)
        right = degree_between(robot_pos, BLUE_GOAL_RIGHT)
    else:
        left = degree_between(robot_pos, RED_GOAL_LEFT)
        right = degree_between(robot_pos, RED_GOAL_RIGHT)

    return [left, right]


def attacking_right():
    """Attacking right."""
    return not attacking_left()


def inside_field(global_position):
    """Inside field."""
    margin = 500
    if abs(global_position.x) > HALF_FIELD_LENGTH + margin or abs(
            global_position.y) > HALF_FIELD_WIDTH + margin:
        return False
    else:
        return True


def inside_view(field_position):
    """Inside view."""
    if abs(field_position.x) > MAX_VIEW_DIST or \
            abs(field_position.y) > MAX_VIEW_DIST:
        return False
    elif field_position.x == 0.0 or field_position.y == 0.0:
        return False
    else:
        return True


def inverse_global_pos_by_side(pos):
    """Convert a pos in left side to red side by mirror."""
    return Vector3(-pos.x, pos.y, angle_normalization(180 - pos.anglez))
