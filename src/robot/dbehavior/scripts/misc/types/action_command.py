"""ActionCommand methods."""
# @Author: Yusu Pan <yuthon>
# @Date:   2017-06-13T11:31:49+08:00
# @Email:  xxdsox@gmail.com
# @Project: humanoid
# @Filename: action_command.py
# @Last modified by:   yuthon
# @Last modified time: 2017-06-13T11:32:06+08:00
# @Copyright: ZJUDancer

from geometry_msgs.msg import Vector3
from dmotion.msg import ActionCmd


def head(cmd, yaw=0, pitch=0, yaw_speed=1, pitch_speed=1):
    """Get HeadCommand."""
    cmd.cmd_head = Vector3(0, pitch, yaw)
    cmd.cmd_head_speed = Vector3(0, pitch_speed, yaw_speed)
    return cmd


def look_at(cmd, yaw=0, pitch=0, yaw_speed=1, pitch_speed=1):
    """Get look_at HeadCommand."""
    cmd.cmd_head = Vector3(0, float(pitch), float(yaw))
    cmd.cmd_head_speed = Vector3(0, float(pitch_speed), float(yaw_speed))
    return cmd


def crouch(cmd):
    """Get CROUCH BodyCommand."""
    return _make_body_command(cmd, ActionCmd.CROUCH)


def standup(cmd):
    """Get STANDUP BodyCommand."""
    return _make_body_command(cmd, ActionCmd.STANDUP)


def wenxi_gaits(cmd):
    """Get WENXI BodyCommand."""
    return _make_body_command(cmd, ActionCmd.WENXI)


def walk(cmd, forward=0, left=0, turn=0):
    """Get walk BodyCommand."""
    return _make_body_command(cmd, ActionCmd.WENXI,
                              forward=forward, left=left, turn=turn)


def goalie_mid(cmd):
    """Get GOALIE BodyCommand."""
    return _make_body_command(cmd, ActionCmd.GOALIE)


def kick(cmd, rightkick=1):
    """Get KICK BodyCommand."""
    cmd = _make_body_command(cmd, ActionCmd.KICK)
    if (rightkick == 1):
        cmd.kick_side = ActionCmd.RIGHT_KICK
    else:
        cmd.kick_side = ActionCmd.LEFT_KICK
    return cmd


def getup_back(cmd):
    """Get SETUPBACK BodyCommand."""
    return _make_body_command(cmd, ActionCmd.SETUPBACK)


def getup_front(cmd):
    """Get SETUPFRONT BodyCommand."""
    return _make_body_command(cmd, ActionCmd.SETUPFRONT)


def _make_body_command(cmd, gait_type, forward=0, left=0, turn=0):
    """Init BodyCommand."""
    cmd.gait_type = gait_type
    cmd.cmd_vel.linear = Vector3(forward, left, 0)
    cmd.cmd_vel.angular = Vector3(0, 0, turn)
    return cmd
