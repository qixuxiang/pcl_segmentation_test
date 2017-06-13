"""Calculate walk engine."""

import copy
from math import pi, cos, sin
from ..utils.mathutil import calc_field_position, sign, angle_normalization
from ..types.vec_pos import VecPos
from ..status.gglobal import get_bb

# __all__ = ['get_walk']

NEARBY_SWITCHING_DISTANCE = 30
FARAWAY_SWITCHING_DISTANCE = 40
TOP_X_MAX = 6.0
TOP_THETA_MAX = 5.0
MID_X_MAX = 4.5
MID_THETA_MAX = 10.0
AD_X_MAX = 2.0
AD_THETA_MAX = 10.0
STEP_THETA_MAX = 15.0

THETA_SAFE = 10.0
epso = 1e-10
walkAbility = [TOP_X_MAX, TOP_X_MAX, MID_X_MAX, AD_X_MAX, epso], \
              [epso, TOP_THETA_MAX, MID_THETA_MAX, AD_THETA_MAX,
               STEP_THETA_MAX]

WALength = len(walkAbility[0])


class WalkToPoint(object):
    """Walk to point.

    input: field pos && attackAngle
    """

    def __init__(self):
        """Init."""
        self.nearDistance = NEARBY_SWITCHING_DISTANCE

    def update(self, dest_pos, dest_angle, robot_ctrl):
        """Update."""
        self.destPos = dest_pos
        self.destAngle = dest_angle
        self.robotCtrl = robot_ctrl

    def run_geom(self, force_all_direc):
        """Run geometry."""
        if self.destPos.get_magnitude() <= self.nearDistance or \
                force_all_direc:
            self.nearDistance = FARAWAY_SWITCHING_DISTANCE
            return self.run_all_direc(self.destPos, self.destAngle)
        else:
            self.nearDistance = NEARBY_SWITCHING_DISTANCE
            return self.run_fast(self.destPos, self.destAngle)

    def getx_max(self, r):
        """Get max sx and max st."""
        a_norm = [[0 for col in range(0, WALength - 1)]
                  for row in range(0, WALength)]
        b = [0 for col in range(0, len(walkAbility[0]) - 1)]

        for i in range(0, 4):
            a_norm[1][i] = walkAbility[1][i + 1] - walkAbility[1][i] + epso
            a_norm[0][i] = -walkAbility[0][i + 1] + walkAbility[0][i] + epso
            b[i] = a_norm[0][i] * walkAbility[1][i] + \
                a_norm[1][i] * walkAbility[0][i]
        rec = [walkAbility[0][i] * 1.0 / walkAbility[1][i]
               * 180 / pi for i in range(0, len(walkAbility[0]))]
        for i in range(0, 4):
            if r <= rec[i]:
                if abs(a_norm[1][i]) < epso:
                    gait_sx = r * pi * walkAbility[1][i] / 180
                    gait_st = walkAbility[1][i]
                else:
                    gait_st = b[i] / a_norm[1][i] / \
                              (pi * r / 180 + a_norm[0][i] / a_norm[1][i])
                    gait_sx = (b[i] - a_norm[0][i] * gait_st) / a_norm[1][i]

        gait_sy = 0
        return gait_sx, gait_sy, gait_st

    def run_all_direc(self, d_pos, d_angle):
        """Run in all direction low speed ok."""
        mirrorflag = False
        m_destPos = copy.copy(d_pos)
        m_destAngle = copy.copy(d_angle)
        if d_pos.y < 0:
            m_destPos.y = -m_destPos.y
            m_destAngle = - m_destAngle
            mirrorflag = True
        low = min(AD_X_MAX, m_destPos.get_magnitude())
        theta_p = m_destPos.get_angle()
        theta_attack = m_destAngle
        # print theta_attack
        if abs(theta_attack) >= AD_THETA_MAX:
            theta_out = AD_THETA_MAX * sign(theta_attack)
        else:
            theta_out = theta_attack
        # print theta_out
        tmp_theta = theta_p - theta_out
        x_out = low * cos(tmp_theta * pi / 180)
        y_out = low * sin(tmp_theta * pi / 180)
        if mirrorflag:
            y_out = -y_out
            theta_out = -theta_out

        return x_out, y_out, theta_out

    def run_fast(self, d_pos, d_angle):
        """Far away run fast."""
        sx_star, sy_star, st_star = 0, 0, 0
        mirror_flag = False
        m_d_pos = copy.copy(d_pos)
        m_d_angle = copy.copy(d_angle)
        if d_pos.y < 0:
            m_d_pos.y = -m_d_pos.y
            m_d_angle = - m_d_angle
            mirror_flag = True

        x_input = self.robotCtrl.gait_sx
        theta_p = m_d_pos.get_angle()
        theta_attack = m_d_angle
        theta_safe = min(theta_p, THETA_SAFE)
        theta_attack_p = theta_attack - theta_p
        a_dest_pos = copy.copy(m_d_pos)
        a_dest_pos.rotate(90)

        if theta_attack_p >= theta_p:
            theta_max = theta_p
        elif theta_attack_p >= theta_safe:
            theta_max = theta_attack_p
        else:
            theta_max = theta_safe
        theta_max = min(theta_max, theta_safe)
        theta_max = min(theta_max, theta_p)

        a_transPos = copy.copy(a_dest_pos)
        a_transPos.rotate(theta_max)
        dt = 1
        stepnum_min = 9999

        i = 0
        cross_i = VecPos(0, 0)
        while i <= theta_max:
            a_i = copy.copy(a_dest_pos)
            a_i.rotate(i)
            b_i = a_i.x * m_d_pos.x + a_i.y * m_d_pos.y

            r_i = b_i / (a_i.y - a_i.get_magnitude() + epso)
            sx, sy, st = self.getx_max(r_i)
            if st >= theta_p:
                st = theta_p
            c_i = i + theta_p
            cross_i.x = r_i * sin(c_i * pi / 180)
            cross_i.y = r_i * (1 - cos(c_i * pi / 180))
            line_i = copy.copy(m_d_pos - cross_i)

            k1 = 15.0
            k2 = 10.0
            stepnum_arc = c_i / (st + epso == 0 and [epso] or [st + epso])[0]
            stepnum_line = line_i.get_magnitude() / TOP_X_MAX
            if theta_p <= AD_THETA_MAX + 20:
                k2 = 25
                stepnum_speed = abs(max(-sx + x_input, 0)) / TOP_X_MAX * \
                    k1 * theta_p * pi / 180 \
                    + abs(TOP_X_MAX - sx) * k2 / TOP_X_MAX
            else:
                stepnum_speed = abs(max(-sx + x_input, 0)) / TOP_X_MAX * \
                    k1 * theta_p * pi / 180 \
                    + abs(TOP_X_MAX - sx) * k2 / TOP_X_MAX

            stepnum = stepnum_arc + stepnum_line + stepnum_speed
            if stepnum_min >= stepnum:
                stepnum_min = stepnum
                sx_star, sy_star, st_star = sx, sy, st

            if abs(theta_max) < TOP_THETA_MAX:
                sx_star, sy_star, st_star = TOP_X_MAX, 0, theta_max
                break
            i += dt

        if mirror_flag:
            st_star = -st_star

        return sx_star, sy_star, st_star


w_instance = WalkToPoint()


def get_walk(dest_global_pos, robot_pos, force_all_direc=False):
    """Get walk."""
    bb = get_bb()
    robot_ctrl = bb.motion.robotCtrl
    dest_pos = calc_field_position(dest_global_pos, robot_pos)
    dest_angle = angle_normalization(dest_global_pos.anglez - robot_pos.anglez)
    w_instance.update(dest_pos, dest_angle, robot_ctrl)
    return w_instance.run_geom(force_all_direc)


def get_walk_field(dest_field_pos, angle, force_all_direc=False):
    """Get walk in field."""
    bb = get_bb()
    robot_ctrl = bb.motion.robotCtrl
    w_instance.update(dest_field_pos, angle, robot_ctrl)
    return w_instance.run_geom(force_all_direc)
