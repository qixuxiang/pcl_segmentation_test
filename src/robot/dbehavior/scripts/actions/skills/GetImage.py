"""GetImage action."""
import rospy
from misc.bt import Action
from misc.utils.timer import Timer
from misc.types.constant import MAX_PLAT_PITCH, MIN_PLAT_PITCH, MAX_PLAT_YAW


class GetImage(Action):
    """GetImage."""

    def init(self):
        """Init."""
        self.timer = Timer(8).start()
        self.initTimer = Timer(50).start()

        self.iter = iter(path)
        self.point = (0, 0)

    def tick(self):
        """Tick."""
        self.crouch()
        if not self.timer.finished():
            pass

        self.lookat(self.point[0], self.point[1])
        if self.timer.finished():
            self.cycle += 1
            self.timer.restart()
            try:
                self.point = self.iter.next()
            except StopIteration:
                # path.reverse()
                self.iter = iter(path)

            # FIXME(corenel) use lower_board_connected
            if self.world.lower_board_connected:  # self.world.uptime > 2:
                self.capture()
                rospy.loginfo("{} Cheese! ({}, {})".format(self.cycle,
                                                           self.point[0],
                                                           self.point[1]))
            else:
                rospy.logerr("Lower board not connected, skip capture")


diff_yaw = [i for i in range(-MAX_PLAT_YAW, MAX_PLAT_YAW, 15)]
diff_pitch = [i for i in range(MIN_PLAT_PITCH, MAX_PLAT_PITCH, 15)]
path = []

for i in range(len(diff_pitch)):
    tmp = [diff_pitch[i] for j in range(len(diff_yaw))]
    path += zip(diff_yaw, tmp)
    diff_yaw.reverse()
