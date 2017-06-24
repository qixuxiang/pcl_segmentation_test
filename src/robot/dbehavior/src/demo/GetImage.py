from zjudancer import *


param = getParam()
diff_yaw = [i for i in range(-param.maxYaw, param.maxYaw, 15)]
diff_pitch = [i for i in range(param.minPitch, param.maxPitch, 15)]
path = []


for i in range(len(diff_pitch)):
    tmp = [diff_pitch[i] for j in range(len(diff_yaw))]
    path += zip(diff_yaw, tmp)
    diff_yaw.reverse()

# Move(1s) --> Keep(1s) --> Capture(1s) --> Move(1s)

class GetImage(Action):
    def init(self):
        self.timer = Timer(2)
        self.initTimer = Timer(5)
        self.cycle = 0
        self.point = (0, 0)
        self.capturing = False
        self.iter = iter(path)

    def tick(self):
        self.crouch()
        if not self.initTimer.finished():
            pass

        self.lookAt(self.point[1], self.point[0])

        if self.timer.finished():
            self.cycle += 1

            self.timer.restart()

currentPoint = (0, 0)
iteration = iter(path)

# wait
timer = Timer(5)
while not timer.finished():
    timer.sleep(1)

@repeatSec(1)
class Move(Action):
    def tick(self):
        self.lookAt(pitch=currentPoint[1], yaw=currentPoint[0])
        return self.running()

@repeatSec(2)
class Keep(Action):
    def tick(self):
        self.lookAt(pitch=currentPoint[1], yaw=currentPoint[0])
        return self.running()

@repeatSec(0.5)
class Capture(Action):
    def init(self):
        global currentPoint
        global iteration
        try:
            currentPoint = iteration.next()
        except StopIteration:
            iteration = iter(path)

    def tick(self):
        # TODO(MWX), Task need onEnter and onLeave
        #print 'Cheese! ({}, {})'.format(currentPoint[1], currentPoint[0])
        self.bb.behaviorInfo.save_image = True
        return self.running()



GetImage = sequence(
    Move, Keep, Capture
)
