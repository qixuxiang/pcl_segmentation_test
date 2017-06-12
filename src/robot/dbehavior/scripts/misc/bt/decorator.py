"""Decorator Node."""

from node import Node, Condition, Status
from util.Timer import Timer


class Decorator(Node):
    """Decorator Node."""

    def __init__(self, child=None):
        """Init decorator node."""
        super(Decorator, self).__init__()
        self.label = '(Decorator)'
        self.shape = 'house'
        if child:
            self.add_child(child)

    def tick(self):
        """Tick decorator node."""
        raise NotImplemented


def repeat_time(runs, second=True):
    """Execute child node N seconds.

    If N = -1, will repeat forever ignoring failure
    :param runs time to repeat
    :param second time unit in second
    """
    def wrap(child):
        class Repeat(Decorator):
            def init(self):
                if second:
                    self.runs = runs * 1000000
                else:
                    self.runs = runs

                self.timer = Timer(runs)
                self.add_child(child)

            def tick(self):
                if self.status is not Status.RUNNING:
                    self.timer.restart()

                if len(self._children) < 1:
                    raise Exception('Repeater should have exactly one child')

                # runs forever
                if self.runs is -1 or self.timer.elapsed() < self.runs:
                    status = self._children[0].tick()

                    if status is Status.RUNNING:
                        return self.running()
                    elif status is Status.FAILURE:
                        return self.failure()
                    elif status is Status.SUCCESS:
                        return self.success()

                else:
                    return self.success()

            def reset_(self):
                self.timer.restart()

        return Repeat

    return wrap


# FIXME, decorator may be only initialized once
def see_ball(child):
    """Check if ball is seen."""
    class SeeBall(Decorator):
        def init(self):
            self.add_child(child)

        def tick(self):
            # try:
            if self.world.see_ball:  # or self.world.mem_ball_valid:
                status = self._children[0].tick()
                if status is Status.RUNNING:
                    return self.running()
                elif status is Status.SUCCESS:
                    return self.success()
                else:
                    return self.failure()
            else:
                self._children[0].failure()
                return self.failure()
            # except:
            #     print 'failure {}'.format(self._children[0].get_name())
            #     return self.failure()

    return SeeBall


def once(child):
    """Execute an task only once util success."""
    class Once(Decorator):
        def init(self):
            self.add_child(child)
            self.executed = False

        def tick(self):
            if not self.executed:
                status = self._children[0].tick()
                if status is Status.RUNNING:
                    return self.running()
                elif status is Status.SUCCESS:
                    self.executed = True
                    return self.success()
                else:
                    return self.failure()
            else:
                return self.status

    return Once


def condition(function):
    """Wrap condition node.

    :param function: condition function
    :return: wrapped condition Node
    """
    def tick(self):
        if function(self):
            return self.success()
        else:
            return self.failure()

    wrapped_condition = type(function.__name__, (Condition,), {'tick': tick})
    return wrapped_condition


def true(child):
    """Return true always."""
    class AlwaysTrue(Decorator):
        def init(self):
            self.add_child(child)

        def tick(self):
            self._children[0].tick()
            return self.success()

    return AlwaysTrue
