from zjudancer import *

class TrackBall(Action):
    def tick(self):
        track = self.bb.visionInfo.ballTrack
        self.lookAt(track.pitch, track.yaw)
