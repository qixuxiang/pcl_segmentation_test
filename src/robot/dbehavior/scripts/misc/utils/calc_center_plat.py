"""Calculate center plat."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-13T10:10:35+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: calc_center_plat.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-13T10:10:41+08:00
# @Copyright: ZJUDancer

import rospy
from ..types.constant import (VIEW_CENTER, MAX_PLAT_YAW,
                             MIN_PLAT_PITCH, MAX_PLAT_PITCH)
from ..types.vec_pos import VecPos
from ..utils.mathutil import get_dis


def calc_center_plat(field):
    """Calc center plat.

    Given ball_field, calculate the right plat angle,
    making the ball in the center of the view
    (min_h, max_h) (min_v, max_v) make four part recursively calculate
    :return VecPos best flat angle
    """
    # start_time = current_milli_time()

    outer = _Area(-MAX_PLAT_YAW, MAX_PLAT_YAW, MIN_PLAT_PITCH, MAX_PLAT_PITCH)
    smallest_distance = 1e10
    good_distance = 10
    inner_loop = 0
    better = 1e10
    best_plat = VecPos(0, 0)

    while smallest_distance > good_distance:
        if inner_loop > 50 or better < 1:
            break
        last_outer = outer
        for area in outer.get_four_part():
            inner_loop += 1
            tmp_dis = get_dis(_calc_img(field, area.center_point), VIEW_CENTER)
            if tmp_dis < smallest_distance:
                better = abs(tmp_dis - smallest_distance)
                best_plat = area.center_point
                smallest_distance = tmp_dis
                outer = area

        if outer is last_outer:
            outer.div(2.0)

    # todo, this is an bug
    if field.x < 0:
        if field.y > 20 and best_plat.x < -50:
            rospy.loginfo("Special plat left")
            best_plat.x += 180
            best_plat.y = 35
        elif field.y < -20 and best_plat.x > 50:
            rospy.loginfo("Special plat right")
            best_plat.x -= 180
            best_plat.y = 35

    # if field.y * best_plat.x < 0:
    #     if best_plat.x > 0:
    #         best_plat.x -= 180
    #     else:
    #         best_plat.x += 180
    #
    #     best_plat.y = 30

    return best_plat


def _calc_img(field, plat):
    """Calc image pos.

    :param field: field position
    :param plat: plat angle
    """
    # TODO(corenel) reimplement it
    tmp = calc_image_position(int(field.x), int(field.y),
                              plat.x, plat.y, 0, 0, 0)
    x = VecPos(tmp.x, tmp.y)
    return x


def _get_center(l, h):
    if not h >= l:
        raise ValueError('high must be larger than low')

    return l + (h - l) / 2.0


class _Area(object):
    """Helper class."""

    def __init__(self, lx, hx, ly, hy):
        self.low_x = lx
        self.low_y = ly
        self.high_x = hx
        self.high_y = hy
        self.center_x = _get_center(self.low_x, self.high_x)
        self.center_y = _get_center(self.low_y, self.high_y)
        self.center_point = VecPos(self.center_x, self.center_y)

        # if not self.low_x <= self.center_x <= self.high_x:
        #     raise Exception('center error')
        # elif not self.low_y <= self.center_y <= self.high_y:
        #     raise Exception('center error')

    def __str__(self):
        return '[{}, {}, {}, {}]'.format(self.low_x, self.high_x,
                                         self.low_y, self.high_y)

    def div(self, other):
        dx = (self.center_x - self.low_x) / other
        dy = (self.center_y - self.low_y) / other
        self.low_x = self.center_x - dx
        self.high_x = self.center_x + dx
        self.low_y = self.center_y - dy
        self.high_y = self.center_y + dy

        self.center_x = _get_center(self.low_x, self.high_x)
        self.center_y = _get_center(self.low_y, self.high_y)
        self.center_point = VecPos(self.center_x, self.center_y)

    def get_four_part(self):
        down_left = _Area(self.low_x, self.center_x,
                          self.low_y, self.center_y)
        down_right = _Area(self.center_x, self.high_x,
                           self.low_y, self.center_y)
        up_left = _Area(self.low_x, self.center_x,
                        self.center_y, self.high_y)
        up_right = _Area(self.center_x, self.high_x,
                         self.center_y, self.high_y)

        return down_left, down_right, up_left, up_right
