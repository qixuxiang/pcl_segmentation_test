"""Set state."""

from misc.bt import Action
from misc.bt import parallel


class _Set(Action):
    def tick(self):
        self.crouch()
        self.lookat(0, 10)
        # print 'Set behaviour ticking'
        return self.success()


Set = parallel(_Set)
