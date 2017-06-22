from inspect import isclass
from Blackboard import getbb
from utils.actioncommand import *

class Task(object):
    """
    Behavior tree Node
    The core class for DecisionMaking.
    """
    def __init__(self):
        # Status Code
        self._RUNNING = 1
        self._FAILURE = 2
        self._SUCCESS = 3

        self._children = []
        self.bb = getbb()
        self.init()

    def init(self):
        """
        This funciton is by default called after Task object initialised.
        You should not define __init__ function is subcalss, if you want
        to add custom variables, override this funciton.
        """
        raise NotImplementedError

    def success(self):
        self.init()
        return self._SUCCESS

    def failure(self):
        self.init()
        return self._FAILURE

    def running(self):
        return self._RUNNING

    def addChild(self, task):
        """
        Add children to this node
        """
        if task is None:
            raise TypeError('Trying to add None as child')
        elif task is self:
            raise TypeError('Trying to add self as child')
        elif not isinstance(task, Task) and not issubclass(task, Task):
            raise TypeError('Not an instance or subclass of Node')

        if isclass(task):
            self._children.append(task())
        else:
            self._children.append(task)

        return self

    def tick(self):
        raise NotImplementedError

class Action(Task):
    """
    Behavior tree leaf, who can change ActionCommand
    """
    def __init__(self):
        super(Action, self).__init__()

    def init(self):
        pass

    def addChild(self, task):
        raise Exception('Actions are leaf node, must not have child')

    def walk(self, x, y, t):
        self.bb.actionCmd.bodyCmd = walk(x, y, t)

    def kickLeft(self):
        self.bb.actionCmd.bodyCmd = kickRight()

    def kickRight(self):
        self.bb.actionCmd.bodyCmd = kickLeft()

    def goto(self, dest):
        x, y, t = getWalk(dest, self.bb.visionInfo.robotPos)

