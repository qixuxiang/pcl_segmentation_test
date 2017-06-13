"""Kick skill.

line up and goal!
"""

import rospy
from .Lineup import Lineup
from .ApproachBall import ApproachBall
from ..headskills.TrackBall import TrackBall
from misc.bt import Action, sequence, parallel
from misc.utils.timer import Timer

DEBUG = False


class _Kick(Action):
    def init(self):
        self.kick_timer = Timer()
        self.kicking = False

    def reset_(self):
        self.kicking = False

    def tick(self):
        self.debug_log()
        self.lookat(0, 20)

        if not self.kicking:
            self.kicking = True
            self.kick_timer.restart()

            closer_to_left_foot = self.world.ball_field.y > 0
            left_kick = self.cfg.LEFT_KICK
            right_kick = self.cfg.RIGHT_KICK

            if left_kick and closer_to_left_foot:
                # rospy.loginfo('Left kick')
                self.kick()
            elif right_kick and not closer_to_left_foot:
                # rospy.loginfo('right kick')
                self.kick(1)
            else:
                return self.failure()

            return self.running()

        elif self.kick_timer.elapsed() < 3000000:

            closer_to_left_foot = self.world.ball_field.y > 0
            left_kick = self.cfg.LEFT_KICK
            right_kick = self.cfg.RIGHT_KICK

            if left_kick and closer_to_left_foot:
                # rospy.loginfo('Left kick')
                self.kick()
            elif right_kick and not closer_to_left_foot:
                # rospy.loginfo('Right kick')
                self.kick(1)
            else:
                return self.failure()

            return self.running()

        else:
            self.lookat(0, 15)
            self.step()

            if self.world.see_ball:  # and self.kick_timer.elapsed() > 6000000:
                self.world.enable_kick = False
                return self.success()
            elif self.kick_timer.elapsed() > 8000000:
                return self.success()
            else:
                return self.running()


Kick = sequence(ApproachBall,
                Lineup,
                _Kick)
