"""Calc Attack Point."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-12T18:10:20+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: calc_attack_point.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-12T18:10:22+08:00
# @Copyright: ZJUDancer

from math import cos, sin, degrees
from geometry_msgs.msg import Vector3
from ..types.vec_pos import VecPos
from ..utils.mathutil import PI, get_dis, degree_between, angle_between
from ..types.field_geometry import get_attack_target


rub = None
final_dest = None
dest = None


def calc_attack_point(enable_kick, robot_pos, ball_pos, ball_field, cfg):
    """Calc Attack Point.

    according to our side, gloabl ball position and robot_pos,
    calculate the next right point we should go.
    :param robot_pos, localized robot global position
    :param ball_pos, localized ball global position
    :return next point to go
    """
    global rub, final_dest, dest
    # todo, change attack goal, we got kick to target
    attack_target = get_attack_target()

    final_dest = _c_final_dest(enable_kick, ball_field,
                               ball_pos, attack_target, cfg)
    rub = _c_rub(ball_pos, attack_target, robot_pos)
    theta = degree_between(cfg.DOGE_POINT, rub)
    theta1 = cfg.doge_angle
    theta2 = -cfg.doge_angle
    # print '{} {} {} {} {}'.format(theta, theta1, theta2, rub, ENEMY_GOAL)

    # avoid blur when theta is close to 180
    if abs(theta) > 170 and rub.x < -10:
        dest = _c_doge_point(rub, robot_pos, ball_pos,
                             attack_target, final_dest, 'up', cfg)
    elif theta > theta1 and rub.x < -10:
        dest = _c_doge_point(rub, robot_pos, ball_pos,
                             attack_target, final_dest, 'up', cfg)
    elif theta < theta2 and rub.x < -10:
        dest = _c_doge_point(rub, robot_pos, ball_pos,
                             attack_target, final_dest, 'down', cfg)
    else:
        dest = final_dest

    # do not go exactly to doge point
    if close_to_point(robot_pos, dest) or \
            close_to_point(robot_pos, final_dest):
        dest = final_dest


def get_attack_result():
    """Get attack result."""
    global final_dest, dest, rub
    return final_dest, dest, rub


def close_to_point(dest, origin):
    """Check if it's close to point."""
    dis = get_dis(dest, origin)
    if dis < 40:
        return True
    else:
        return False


def get_rub():
    """Get rub.

    rub means robot under ball coord
    get robot position under coord of ball,
    the direction of x-axis is from Goal to ball, origin is ball
    """
    global rub
    if not rub:
        raise Exception('rub not initialized, call calc_attack_point first')
    return rub


def _c_final_dest(enable_kick, ball_field, ball_pos,
                  target, cfg, lineup=False):
    """Calculate final dest."""
    theta = angle_between(target, ball_pos)
    closer_to_left_foot = ball_field.y > 0

    if not enable_kick:
        if closer_to_left_foot:
            KICK_POINT = cfg.LEFT_KICK_POINT
        else:
            KICK_POINT = cfg.RIGHT_KICK_POINT

    else:
        can_left_kick = cfg.LEFT_KICK
        can_right_kick = cfg.RIGHT_KICK

        if not can_left_kick:
            KICK_POINT = cfg.RIGHT_KICK_POINT
        elif not can_right_kick:
            KICK_POINT = cfg.LEFT_KICK_POINT
        else:
            if closer_to_left_foot:
                KICK_POINT = cfg.LEFT_KICK_POINT
            else:
                KICK_POINT = cfg.RIGHT_KICK_POINT

    final_x = ball_pos.x + KICK_POINT.x * cos(theta)
    final_y = ball_pos.y + KICK_POINT.x * sin(theta)

    theta2 = angle_between(target, VecPos(final_x, final_y))

    return Vector3(final_x, final_y, degrees(theta2 + PI))


def _c_rub(ball_pos, target, robot_pos):
    """Calculate robot_pos under ball coord."""
    r = VecPos(robot_pos.x, robot_pos.y)
    b = VecPos(ball_pos.x, ball_pos.y)
    b2r = r - b
    theta = degree_between(target, ball_pos)
    b2r.rotate(-theta)
    return b2r


def _c_doge_point(rub, robot_pos, ball_pos, target, final_dest, side, cfg):
    """Calc doge point."""
    DOGE_POINT_UP = cfg.DOGE_POINT_UP
    DOGE_POINT_DOWN = cfg.DOGE_POINT_DOWN

    if side is 'up':
        doge = DOGE_POINT_UP
    else:
        doge = DOGE_POINT_DOWN

    if -30 < rub.x < 0 and abs(rub.y) < 20:
        if side is 'up':
            doge = VecPos(0, 20)
        else:
            doge = VecPos(0, -20)

    theta = angle_between(ball_pos, target)
    dogex = ball_pos.x + doge.y * sin(theta) - doge.x * cos(theta)
    dogey = ball_pos.y - doge.y * cos(theta) + doge.x * sin(theta)
    d = VecPos(dogex, dogey)
    angle = degree_between(d, final_dest)

    return Vector3(dogex, dogey, angle)
