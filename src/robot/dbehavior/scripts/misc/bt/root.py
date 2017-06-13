"""Root Node."""

import rospy
from .node import Node
from ..types.constant import STATE_INITIAL, STATE_READY
from ..status.gglobal import get_req, get_world
from ..blackboard.gc_bb import get_gc


class Root(Node):
    """Root Node."""

    def init(self):
        """Init root node."""
        self.label = '(root)'
        self.shape = 'diamond'
        self.gc = get_gc()
        self.req = get_req()
        self.world = get_world()
        self.first_reentry = True
        self.cycle = 0

    def tick(self):
        """Tick."""
        rospy.logdebug('====== Root tick ========== {} '.format(self.cycle))
        self.cycle += 1

        self.check_reentry()

        if len(self._children) is not 1:
            raise Exception('Root should have only 1 child')

        self._children[0].tick()
        self.update_graph()

    def update_graph(self):
        """Update graphviz."""
        if self.bb.dashmean.updateGraph:
            self.req.btGraph = self.generate_dot()

    def check_reentry(self):
        """Check re-entry."""
        uptime = self.world.uptime
        secs_since_start = self.gc.secsSinceGameStart

        if self.world.lower_board_connected and self.gc.connected and \
                (self.gc.state is STATE_INITIAL or
                 self.gc.state is STATE_READY):
            self.first_reentry = False

        if self.gc.penalised and 0 < self.gc.secsTillUnpenalised < 15:
            rospy.loginfo('Enable re-entry 1')
            self.world.enable_reentry = True

        elif self.first_reentry and uptime + 10 < secs_since_start:
            print 'enable reentry 2'
            rospy.loginfo('Enable re-entry 2')
            self.first_reentry = False
            self.world.enable_reentry = True

# should be done in root state

# state other -> state penalised -> other state
# reset self localization according to compass angle after got play signal and
# lower_board connected

# if lower board connected and STATE_PLAYING and (last state is Penalised):
# if compass angle is close to 90, set to down_point
# else if compass angle is close to -90, set to up_point
# else wait
