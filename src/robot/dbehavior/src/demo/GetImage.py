from zjudancer import *

class GetImage(Action):
    def init(self):
        self.timer = Timer(8)
        self.initTimer = Timer(5)
        self.cycle = 0
        self.point = (0, 0)
        self.iter = iter(path)

    def tick(self):
        self.crouch()
        if not self.timer.finished()
            pass

        self.lookAt(self.point[0], self.point[1])

        if self.timer.finished():
            self.cycle += 1

            self.timer.restart()
            try:
                self.point = self.iter.next()
            except StopIteration:
                self.iter = iter(path)




param = getParam()

diff_yaw = [i for i in range(-param.maxYaw, -param.maxYaw, 15)]
diff_pitch = [i for i in range(param.minPitch, param.maxPitch, 15)]
path = []

for i in range(len(diff_pitch)):
    tmp = [diff_pitch[i] for j in range(len(diff_yaw))]
    path += zip(diff_yaw, tmp)
    diff_yaw.reverse()