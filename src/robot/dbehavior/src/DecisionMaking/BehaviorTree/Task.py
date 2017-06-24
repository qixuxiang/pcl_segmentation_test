# TODO(MWX):
# onEnter and onLeave logic

from enum import Enum
from inspect import isclass
from Blackboard import getbb
from utils.actioncommand import *


class Status(Enum):
    """
    Wrap up all status we need.
    Fresh: the task has never run or has been reset
    Running: the task has not completed and needs to run again
    Failure: the task returned a failure result
    Success: the task returned a success result
    """
    FRESH = 0
    RUNNING = 1
    FAILURE = 2
    SUCCESS = 3

class Task(object):
    """
    Behavior tree Node
    The core class for DecisionMaking.
    """
    def __init__(self):
        # Status Code
        self.status = Status.FRESH

        self._children = []
        self.bb = getbb()
        self.init()


    def init(self):
        """
        This funciton is by default called after Task object initialised.
        You should not define __init__ function is subcalss, if you want
        to add custom variables, override this funciton.
        """
        pass

    def isFresh(self):
        return self.status is Status.FRESH

    def onEnter(self):
        """
        Called on enter this task
        """
        pass

    def onLeave(self):
        """
        Called on leaeve this task
        """
        pass

    def success(self):
        self.init()
        self.status = Status.FRESH
        self.onLeave()
        return self.status

    # FIXME(MWX): status code needs more clear definition
    def failure(self):
        self.init()
        self.status = Status.FAILURE
        self.onLeave()
        return self.status

    def running(self):
        self.status = Status.RUNNING
        return self.status

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

    def do(self, cmd):
        self.bb.actionCmd.bodyCmd = cmd

    def walk(self, forward = 0, left = 0, turn = 0):
        self.bb.actionCmd.bodyCmd = walk(x, y, t)

    def crouch(self):
        self.bb.actionCmd.bodyCmd = crouch()

    def lookAt(self, pitch = 0, yaw = 0):
        self.bb.actionCmd.headCmd = head(pitch, yaw)

    def kickLeft(self):
        self.bb.actionCmd.bodyCmd = kickRight()

    def kickRight(self):
        self.bb.actionCmd.bodyCmd = kickLeft()

    def capture(self):
        self.bb.behaviorInfo.save_image = True

    def goto(self, dest):
        x, y, t = getWalk(dest, self.bb.visionInfo.robotPos)

