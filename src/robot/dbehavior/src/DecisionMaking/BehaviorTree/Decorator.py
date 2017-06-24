from Task import Task, Status
from Timer import Timer

def repeatSec(runs):
    """
    Executes child node N seconds, N must be positive
    """

    def wrap(child):
        class Repeat(Task):
            def __init__(self):
                super(Repeat, self).__init__()
                self.timer = Timer(runs)
                self.addChild(child)

            def tick(self):
                if self.status is not Status.RUNNING:
                    self.timer.restart()

                if len(self._children) is not 1:
                    raise Exception('Repeater should have one child, but I have {}'.format(len(self._children)))

                if not self.timer.finished():
                    self.status =  self._children[0].tick()
                    return self.status
                else:
                    self._children[0].success()
                    return self.success()

        return Repeat

    return wrap

