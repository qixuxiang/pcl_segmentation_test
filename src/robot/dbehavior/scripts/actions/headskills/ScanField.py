"""ScanField headskill."""

from ...misc.bt import Action
from ...misc.utils.timer import Timer
from ...misc.types.vec_pos import VecPos

GazePlats = [
    VecPos(60, 15),
    VecPos(0, 12),
    VecPos(-60, 15),
    VecPos(0, 55)
]
# todo, change this for taller robot


class ScanField(Action):
    """ScanField."""
    
    def init(self):
        """Init."""
        self.iter = iter(GazePlats)
        self.curPlat = self.iter.next()
        self.timer = Timer()

    def tick(self):
        """Tick."""
        if self.timer.elapsed() > 1000000:
            self.timer.restart()
            self.next_plat()

        self.lookat(self.curPlat.x, self.curPlat.y, 10, 10)
        return self.running()

    def next_plat(self):
        """Next plat."""
        try:
            self.curPlat = self.iter.next()
        except StopIteration:
            GazePlats.reverse()
            self.iter = iter(GazePlats)
            self.iter.next()
            self.curPlat = self.iter.next()
