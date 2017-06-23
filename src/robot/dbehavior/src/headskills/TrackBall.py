from zjudancer import *

class TrackBall(Action):
    def tick(self):
        print 'field', self.bb.visionInfo.ball_field
        print 'global', self.bb.visionInfo.ball_global
        track =self.bb.visionInfo.ballTrack
        print track
        self.lookAt(track.pitch, track.yaw)
