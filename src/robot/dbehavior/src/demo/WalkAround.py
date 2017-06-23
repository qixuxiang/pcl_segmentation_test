from zjudancer import *

class WalkAround(Action):
    def tick(self):
        self.walk(2, 0, 5)
        self.lookAt(30, 0)
