"""SearchGoal headskill."""
from ...misc.bt import Action
from ...misc.bt import sequence, selector
from ...misc.utils.timer import Timer
from ...misc.types.vec_pos import VecPos

# first look at localized goal position, until see both goal
# calc another goal


class SearchGoalSearch(Action):
    """SearchGoalSearch."""

    def init(self):
        """Init."""
        self.cycle = 0
        self.scan_timer = Timer()
        self.direction = 'left'

    def reset_(self):
        """Reset."""
        self.world.enable_checkgoal = False
        self.cycle = 0

    def tick(self):
        """Tick."""
        print 'SearchGoalSearch ', self.cycle
        self.enable_localization()

        # print 'SearchGoalSearch {}'.format(self.cycle)
        self.crouch()
        self.cycle += 1

        # self.world.enable_checkgoal = True
        # self.world.check_goal = True
        # print '------------- search goal -------------------'
        # print 'search goal search ', self.world.check_goal

        if not self.world.check_goal:
            print 'world checkgoal disabled'
            return self.success()

        if self.world.see_both_goal or self.world.see_unknown_goal:
            print 'saw both goal'
            self.world.lost_goal_cycle = 0
            return self.failure()

        elif self.cycle > 60:
            print 'cycle > 70'
            self.world.check_goal = False
            return self.success()

        elif self.cycle > 40:

            self.lookat(0, 60, 10, 10)
            if self.world.see_ball and self.world.ball_valid_cycle > 5:
                print 'see ball cycle > 40'
                self.world.lost_goal_cycle = 0
                self.world.check_goal = False
                return self.success()
            else:
                print 'running 1'
                return self.running()

        else:
            print 'running'
            y = self.world.robot_pos.y
            angle = self.world.field_angle

            if abs(angle) < 90:
                if y > 150:
                    self.lookat(-45, 15)
                elif 150 > y > -150:
                    self.lookat(0, 15)
                else:
                    self.lookat(45, 15)
            else:
                if y > 150:
                    self.lookat(45, 15)
                elif 150 > y > -150:
                    self.lookat(0, 15)
                else:
                    self.lookat(-45, 15)

            return self.running()


class TrackGoal(Action):
    """TrackGoal."""

    def init(self):
        """Init."""
        self.hold = 0
        self.cycle = 0
        self.stable_cycle = 0

    def tick(self):
        """Tick."""
        self.enable_localization()
        # Log.debug('TrackGoal tick')
        print 'TrackGoal {}'.format(self.cycle)
        self.crouch()
        self.cycle += 1

        # if not self.world.enable_checkgoal:
        #     return self.success()

        if self.cycle > 40:
            self.world.lost_goal_cycle = 0
            self.lookat(0, 60, 10, 10)

            if self.world.see_ball and self.world.ball_valid_cycle > 10:
                self.world.check_goal = False
                return self.success()
            elif self.cycle > 60:
                self.world.check_goal = False
                return self.success()
            else:
                return self.running()

        else:
            if self.stable_cycle > 10:
                # print 'stable cycle > 5'
                self.lookat(0, 60, 10, 10)
                if self.world.see_ball and self.world.ball_valid_cycle > 5:
                    self.world.check_goal = False
                    # print 'exit on see ball'
                    return self.success()
                else:
                    # print 'running 1'
                    return self.running()

            if self.world.see_both_goal:
                self.stable_cycle += 1
                angle = self.world.field_angle
                if abs(angle) < 90:
                    # print 'gaze red goal'
                    center = self.world.red_goal_center
                else:
                    # print 'gaze blue goal'
                    center = self.world.blue_goal_center

                self.gaze_at(center)
                return self.running()

            elif self.world.see_unknown_goal:
                print 'see unknown goal {}'.format(self.world.unknown_goal)
                up, down = self.calc_up_goal()
                self.hold += 1
                if self.hold < 30:
                    self.gaze_at(up)
                elif self.hold < 60:
                    self.gaze_at(down)
                elif self.hold == 60:
                    self.hold = 0
                return self.running()
            else:
                # print 'nothing'
                self.stable_cycle = 0
                self.gaze_at(VecPos(300, 0))
                return self.running()

    def calc_up_goal(self):
        """Calc up goal."""
        unknown = self.world.unknown_goal
        unknown = VecPos(unknown.x, unknown.y)
        field_angle = self.world.field_angle

        up_vec = VecPos(0, 90).rotate(-field_angle)
        down_vec = VecPos(0, -90).rotate(-field_angle)

        up_goal = unknown + up_vec
        down_vec = unknown + down_vec

        return up_goal, down_vec

    def reset_(self):
        """Reset."""
        self.stable_cycle = 0
        self.cycle = 0
        self.hold = 0


SearchGoal = selector(SearchGoalSearch, TrackGoal)
