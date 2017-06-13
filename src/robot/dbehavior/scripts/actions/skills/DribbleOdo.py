"""DribbleOdo skill."""

from misc.bt import Action, see_ball

DRIBBLE_THRES = 30
EPSO = 1e-10

DRIBBLE_SAFE = -20
DRIBBLE_X_B = -2
DRIBBLE_X_AD_B = -3  # 2
DRIBBLE_X_AD_F = 3  # 2
DRIBBLE_X_MID = 2
DRIBBLE_X_TOP = 3

DRIBBLE_Y_B = 1
DRIBBLE_Y_AD_B = 2
DRIBBLE_Y_AD_F = 2
DRIBBLE_Y_MID = 1
DRIBBLE_Y_TOP = EPSO

STEP_L = 12
STEP_R = -12
RM_L = 3.5
RM_R = -15.5
LM_L = 15.5
LM_R = -3.5

x_can = [DRIBBLE_X_B, DRIBBLE_X_B, DRIBBLE_X_AD_B, 0, DRIBBLE_X_AD_F,
         DRIBBLE_X_MID, DRIBBLE_X_TOP, DRIBBLE_X_TOP]
y_can = [EPSO, DRIBBLE_Y_B, DRIBBLE_Y_B, DRIBBLE_Y_AD_B, DRIBBLE_Y_AD_F,
         DRIBBLE_Y_MID, DRIBBLE_Y_TOP, EPSO]
rec_can = [x_can[i] * 1.0 / y_can[i] for i in range(0, len(x_can))]

DEBUG_DRIBBLE = True


# todo, small dribble and large dribble
@see_ball
class DribbleOdo(Action):
    """DribbleOdo."""

    def init(self):
        """Init."""
        self.sx, self.sy = 0, 0
        self.odo_x = 0
        self.odo_y = 0

    def tick(self):
        """Tick."""
        self.lookat(0, 60)

        delta = self.world.deltaData

        self.odo_x += delta.x
        self.odo_x += delta.y

        print 'Dribble odo {} {} '.format(self.odo_x, self.odo_y)

        if not self.world.see_ball:
            self.step()
            return self.failure()

        elif abs(self.odo_x) > 80:
            return self.success()

        else:
            vy = self.world.vy
            if vy <= 0:
                current_l = STEP_L - (STEP_L - RM_L) / 1.8 * abs(vy)
                current_r = STEP_R - (STEP_R - RM_R) / 1.8 * abs(vy)
            else:
                current_l = STEP_L - (STEP_L - LM_L) / 1.8 * abs(vy)
                current_r = STEP_R - (STEP_R - LM_R) / 1.8 * abs(vy)

            eye_y = (current_l + current_r) / 2.0

            ball_y = self.world.ball_field.y
            ball_x = self.world.ball_field.x
            diff_y = ball_y - eye_y

            if STEP_L > diff_y > STEP_R:
                return self.failure()
            else:
                diff_x = ball_x + 10  # FIXME

                theta = diff_x / abs(diff_y + 0.00001)

                for i in range(0, 7):
                    if rec_can[i] <= theta <= rec_can[i + 1]:
                        a_s = (theta * y_can[i] - x_can[i]) / \
                            (x_can[i + 1] - x_can[i] -
                             theta * (y_can[i + 1] - y_can[i]))
                        self.sx = x_can[i] + a_s * (x_can[i + 1] - x_can[i])
                        self.sy = y_can[i] + a_s * (y_can[i + 1] - y_can[i])
                    if diff_x >= 0:
                        self.sx = min(self.sx, diff_x)
                    else:
                        self.sx = max(self.sx, diff_x)
                    self.sy = min(abs(diff_y), self.sy)
                if diff_y < 0:
                    self.sy = -self.sy

                t = 0

                self.walk(self.sx, self.sy, t)
                return self.running()
