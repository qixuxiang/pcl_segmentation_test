"""Branch Node.

Including selector, ev_selector, sequence, ev_sequence and parallel.
"""

# Node tick must return status
from node import Node, Status


def selector(*args):
    """Retuen Selector Node."""
    class Selector(Node):
        """
        Runs children in order until one succeeds or running.

        if all fails, return Failure
        """

        def __init__(self):
            super(Selector, self).__init__()
            self.run_index = 0
            # self.label = '(Selector)'
            for child in args:
                self.add_child(child)

        def tick(self):
            """Tick until some child returns not failure.

            if status is running, then next time tick from here
            :return:
            """
            if len(self._children) is 0:
                return self.success()

            for index in range(self.run_index, len(self._children)):
                status = self._children[index].tick()

                if status is None:
                    raise Exception('Node tick not returning status {}'.format(
                        self._children[index].get_name()))

                # we remember the index of the running child
                # and next time when this node got ticked,
                # we start from the running child
                if status is Status.RUNNING:
                    self.run_index = index
                    return self.running()
                elif status is Status.SUCCESS:
                    self.run_index = 0
                    return self.success()

            # We run out of children, so return failure
            # and go to the first child

            self.reset()
            self.run_index = 0
            return self.failure()

    return Selector


def ev_selector(*args):
    """Event Selector."""
    class EvSelector(Node):
        """Event selector, run every tick from index 0.

        Runs children in order until one succeeds or running,
        if all fails, return Failure.
        """

        def __init__(self):
            super(EvSelector, self).__init__()
            self.label = '(EvSelector)'
            for child in args:
                self.add_child(child)

        def tick(self):
            """Tick until some child returns not failure.

            if status is running, then next time tick from here
            :return:
            """
            if len(self._children) is 0:
                return self.success()

            for index in range(len(self._children)):
                status = self._children[index].tick()

                if status is None:
                    raise Exception('Node tick not returning status {}'.format(
                        self._children[index].get_name()))

                if status is Status.RUNNING:
                    return self.running()
                elif status is Status.SUCCESS:
                    return self.success()

            # We run out of children, so return failure
            # and go to the first child
            return self.failure()

    return EvSelector


def sequence(*args):
    """Sequence."""
    class Sequence(Node):
        def __init__(self):
            super(Sequence, self).__init__()
            self.label = '(Sequence)'
            self.run_index = 0
            for child in args:
                self.add_child(child)

        def tick(self):
            if self.num_children is 0:
                return self.success()

            for index in range(self.run_index, len(self._children)):
                status = self._children[index].tick()

                if status is None:
                    raise Exception('Node tick not returning status {}'.format(
                        self._children[index].get_name()))

                if status is Status.RUNNING:
                    self.run_index = index
                    return self.running()
                elif status is Status.FAILURE:
                    self.run_index = 0
                    return self.failure()

            self.run_index = 0
            self.reset()
            return self.success()

    return Sequence


def ev_sequence(*args):
    """Event Sequence."""
    class EvSequence(Node):
        def __init__(self):
            super(EvSequence, self).__init__()
            self.label = '(EvSequence)'
            self.run_index = 0
            for child in args:
                self.add_child(child)

        def tick(self):
            if self.num_children is 0:
                return self.success()

            for index in range(len(self._children)):
                # loop child, if one succeeds, try next
                status = self._children[index].tick()

                if status is None:
                    raise Exception('Node tick not returning status {}'.format(
                        self._children[index].get_name()))

                if status is Status.RUNNING:
                    return self.running()
                elif status is Status.FAILURE:
                    return self.failure()

            return self.success()

    return EvSequence


def parallel(*args):
    """Parallel."""
    class Parallel(Node):
        """Executes all children close to simultaneously.

        If any fail, returns Failure. Once all succeed, returns SUCCESS.
        """

        def __init__(self):
            super(Parallel, self).__init__()
            self.label = '(Parallel)'
            self.num_success = 0
            for child in args:
                self.add_child(child)

        def reset_(self):
            self.num_success = 0

        def tick(self):
            for child in self._children:
                status = child.tick()

                if status is None:
                    raise Exception('Node tick not returning status {}'.format(
                        child.get_name()))

                if status is Status.SUCCESS:
                    self.num_success += 1
                elif status is Status.FAILURE:
                    return self.failure()

            # if all child returns success, then return success
            if self.num_success is len(self._children):
                return self.success()
            else:
                self.reset_()
                return self.running()

    return Parallel


if __name__ == '__main__':
    pass
