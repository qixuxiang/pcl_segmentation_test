"""VecPos."""

from math import sqrt, atan2, pi, sin, cos, radians

MAX_DIS = 500


class VecPos(object):
    """VecPos class."""

    def __init__(self, x, y):
        """Init VecPos."""
        self.x = x
        self.y = y

    def __str__(self):
        """String for VecPos."""
        return "[{}, {}]".format(self.x, self.y)

    def __add__(self, pos_):
        """Add operation for VecPos."""
        tmp = VecPos(0, 0)
        tmp.x = self.x + pos_.x
        tmp.y = self.y + pos_.y
        return tmp

    def __sub__(self, pos_):
        """Sub operation for VecPos."""
        tmp = VecPos(0, 0)
        tmp.x = self.x - pos_.x
        tmp.y = self.y - pos_.y
        return tmp

    def __isub__(self, other):
        """Isub operation for VecPos."""
        self.x -= other.x
        self.y -= other.y

    def __eq__(self, pos_):
        """Equal operation for VecPos."""
        tmp = VecPos(0, 0)
        tmp.x = pos_.x
        tmp.y = pos_.y
        return tmp

    def get_magnitude(self):
        """Get magnitude."""
        return sqrt(self.x * self.x + self.y * self.y)

    def get_distance(self, dest):
        """Get distance."""
        return sqrt((dest.x - self.x) * (dest.x - self.x) +
                    (dest.y - self.y) * (dest.y - self.y))

    def get_angle(self):
        """Get angle."""
        return atan2(self.y, self.x) * 180 / pi

    def copy(self):
        """Copy operation for VecPos."""
        return VecPos(self.x, self.y)

    def rotate(self, angle):
        """Rotate operation for VecPos."""
        x = self.x
        y = self.y
        self.x = x * cos(radians(angle)) - y * sin(radians(angle))
        self.y = x * sin(radians(angle)) + y * cos(radians(angle))
        return self


def make_vecpos(pos):
    """Genarate VecPos."""
    return VecPos(pos.x, pos.y)
