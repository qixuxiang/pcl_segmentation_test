from zjudancer import *

class TrackBall(Action):
    def tick(self):
        track = self.bb.visionInfo.ballTrack
        #self.lookAt(track.pitch, track.yaw)

        self.lookAt(pitch= 20, yaw=0)
        print self.bb.visionInfo.ball_field
