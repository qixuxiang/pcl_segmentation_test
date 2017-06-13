"""ReEntry skill."""

import rospy
from ..headskills.ScanField import ScanField
from ...misc.bt import Action, parallel
from ...misc.utils.field_geometry import (UP_PICKUP_ENTRY_POINT,
                                          DOWN_PICKUP_ENTRY_POINT)

sc = ScanField()


class _ReEntry(Action):
    def init(self):
        self.cycle = 0
        self.up_cycle = 0
        self.down_cycle = 0

    def reset_(self):
        self.cycle = 0
        self.up_cycle = 0
        self.down_cycle = 0

    def tick(self):
        if not self.world.enable_reentry:
            return self.success()

        else:
            self.crouch()
            angle = self.world.field_angle

            if not self.world.lower_board_connected:
                return self.running()

            else:
                if self.world.see_ball:
                    self.gaze_ball()
                else:
                    sc.tick()

                if not self.gc.penalised:
                    print 'reentry {}'.format(self.cycle)

                    if 20 < angle < 160:
                        self.down_cycle += 1
                    elif -20 > angle - 160:
                        self.up_cycle += 1
                    else:
                        self.cycle += 1

                    if self.down_cycle > 80 and \
                            self.down_cycle - self.up_cycle > 80:
                        self.set_position(DOWN_PICKUP_ENTRY_POINT)
                        rospy.loginfo('[ReEntry] reset position to DOWN_PICK_UP_ENTRY_POINT')
                        self.world.reentry = True
                        self.world.enable_reentry = False
                        return self.success()

                    elif self.up_cycle > 80 and \
                            self.up_cycle - self.down_cycle > 80:
                        self.set_position(UP_PICKUP_ENTRY_POINT)
                        rospy.loginfo('[ReEntry] reset position to UP_PICK_UP_ENTRY_POINT')
                        self.world.reentry = True
                        self.world.enable_reentry = False
                        return self.success()

                    elif self.cycle > 160:
                        self.set_position(UP_PICKUP_ENTRY_POINT)
                        rospy.loginfo('[ReEntry] reset position to UP_PICK_UP_ENTRY_POINT')
                        self.world.reentry = True
                        self.world.enable_reentry = False
                        return self.success()

                    else:
                        return self.running()

                else:
                    return self.running()


ReEntry = _ReEntry
