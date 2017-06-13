"""Finish state."""

from misc.bt import Action
from misc.bt import parallel


class _Finish(Action):
    def tick(self):
        # print 'Finish behaviour ticking'
        self.stand()
        return self.success()


Finish = parallel(_Finish)
