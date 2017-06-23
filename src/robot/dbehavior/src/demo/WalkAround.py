from zjudancer import *

class WalkAround(Action):
    def tick(self):
        self.walk(2, 0, 5)
        track = self.bb.visionInfo.ballTrack
        self.lookAt(track.pitch, track.yaw)
