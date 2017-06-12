"""Math Utilities."""

from math import pi, sqrt, atan2, degrees
from ..types.vec_pos import VecPos

PI = pi
PI_2 = pi / 2.0
PI_4 = pi / 4.0
PI_1_3 = pi / 3.0


def get_angle(vector):
    """Get angle of a vector."""
    return degrees(atan2(vector.y, vector.x))


def get_dis(x, y):
    """Get distance from two points."""
    dx = x.x - y.x
    dy = x.y - y.y
    return sqrt(dx * dx + dy * dy)


def get_magnitude(vec):
    """Get magnitude of a vector."""
    return sqrt(vec.x * vec.x + vec.y * vec.y)


def angle_between(a, b):
    """Angle between two points."""
    return atan2(b.y - a.y, b.x - a.x)


def degree_between(a, b):
    """Degree between two points."""
    return angle_normalization(degrees(angle_between(a, b)))


def angle_between2(a, b, o):
    """Angle between two points related with origin point."""
    return angle_between(a - o, b - o)


def degree_between2(a, b, o):
    """Degree between two points related with origin point."""
    return angle_normalization(degree_between2(a, b, o))


def angle_normalization(angle):
    """Normalize angle."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def abs_angle_diff(angle):
    """Get absolute diff of angle."""
    return abs(angle_normalization(angle))


def sign(v):
    """Sign function."""
    if v > 0:
        return 1.0
    if v < 0:
        return -1.0
    return 0


def calc_global_position(field_pos, robot_state):
    """Get object's global position.

    From robot's global position and object's field position
    """
    tmp = VecPos(field_pos.x, field_pos.y)
    tmp.rotate(robot_state.anglez)
    return VecPos(tmp.x + robot_state.x, tmp.y + robot_state.y)


def calc_field_position(global_pos, robot_state):
    """
    Get the field position of an object.

    From it's global position and robot's global position
    """
    tmp = VecPos(global_pos.x - robot_state.x, global_pos.y - robot_state.y)
    tmp.rotate(-robot_state.anglez)
    return tmp
