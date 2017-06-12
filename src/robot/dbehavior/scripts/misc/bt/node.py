"""Defines Task base class.

Including Node, Action and Condition.
"""

import random
import string
import rospy
from enum import Enum
from inspect import isclass


from ..gglobal import get_req, get_bb, get_world, get_team, get_cfg
from ..utils.mathutil import get_dis, abs_angle_diff, get_magnitude, get_angle
from ..utils.calc_walk_engine import get_walk, get_walk_field
from ..types.vec_pos import VecPos
from util.GameControl import get_gc
from util.actioncommand import walk, look_at, crouch, \
    standup, wenxi_gaits, kick
from util.calc_center_plat import calc_center_plat
from util.list_to_gaitvec import list_to_gaitvec


# color for graphviz
Red = '"#d9534f"'
White = '"#f9f9f9"'
LightBlue = '"#5bc0de"'
Blue = '"#428bca"'
Green = '"#5cb85c"'
Yellow = '"#FFFFE0"'
LightGrey = '"#D3D3D3"'


def random_string(length):
    """Generate random string for naming nodes."""
    pool = string.letters + string.digits
    return ''.join(random.choice(pool) for i in xrange(length))


class Status(Enum):
    """Wrap up all status we need.

    Fresh: the task has never run or has been reset
    Running: the task has not completed and needs to run again
    Failure: the task returned a failure result
    Success: the task returned a success result
    """

    FRESH = 0
    RUNNING = 1
    FAILURE = 2
    SUCCESS = 3


def check_node_method(method):
    """Ensure the node to be appended to tree is exactly a Node, and not self.

    :param method: method to be wrapped
    """
    def wrapped(self, node, *args):
        if node is None:
            return
        if not isinstance(node, Node) and not issubclass(node, Node):
            raise TypeError('Not an instance/subclass of Node')
        elif node is self:
            raise ValueError('Trying to add self as a child')
        else:
            return method(self, node, *args)
    return wrapped


class Node(object):
    """Behavior tree Node.

    The core class for BT nodes.
    """

    def __init__(self):
        """Init Node class."""
        self.status = Status.FRESH
        self.cycle = 0
        self.parent = None
        self.num_children = 0
        self._children = []
        # init name
        self.name = '{}_{}'.format(self.__class__.__name__, random_string(3))
        # every node has the same blackboard
        self.bb = get_bb()
        self.world = get_world()
        self.cfg = get_cfg()
        self.team = get_team()
        # shape for graphviz
        self.color = LightGrey
        self.shape = 'diamond'
        self.label = '(root)'

        self.init()

    def init(self):
        """Init for Node subclass."""
        pass

    def update_bb(self, blackboard):
        """Update blackboard."""
        self.bb = blackboard
        for child in self._children:
            child.update_bb(blackboard)

    def set_status(self, status):
        """Set the status of node."""
        # TODO: enum in ununtu differ from mac
        # if not isinstance(status, Status):
        #     raise TypeError('not Status type')

        # if status not in [status.FAILURE, status.FRESH,
        #                   status.RUNNING, status.SUCCESS]:
        #     raise TypeError('not Status type')
        self.status = status
        return status

    # set the status of self, and return it
    def success(self):
        """Set the SUCCESS status of self, and return it."""
        self.color = Blue
        self.status = Status.SUCCESS
        self.reset()
        return Status.SUCCESS

    def failure(self):
        """Set the FAILURE status of self, and return it."""
        self.color = Red
        self.status = Status.FAILURE
        self.reset()
        return Status.FAILURE

    def running(self):
        """Set the RUNNING status of self, and return it."""
        self.color = Green
        self.status = Status.RUNNING
        return Status.RUNNING

    @check_node_method
    def add_child(self, child_to_add):
        """Add a child to this node.

        :param child_to_add: the child to add
        :return self
        """
        if isclass(child_to_add):
            child = child_to_add()
            self._children.append(child)
            child.parent = self
        else:
            self._children.append(child_to_add)
            child_to_add.parent = self

        self.num_children += 1
        return self

    def add_children(self, *args):
        """Add children to this node."""
        for child in args:
            self.add_child(child)
        return self

    @check_node_method
    def insert_child(self, child_to_insert, index):
        """Insert a child at a position in the existing children of this node.

        :param index: the position to insert the child
        :param child_to_insert: the node to insert
        :return: self
        """
        child_to_insert.parent = self
        self._children.insert(index, child_to_insert)
        return self

    # TODO: to be implemented if useful
    @check_node_method
    def replace_child(self, replacement_child, existing_child):
        """Relpace a existing child with another.

        :param replacement_child: the child for replacement
        :param existing_child: the existing child to be replaced
        :return: self
        """
        return self

    def remove_child(self, child_to_remove):
        """Remove a child by reference.

        :param child_to_remove: reference to the child to remove
        :return: self
        """
        if child_to_remove in self._children:
            index = self._children.index(child_to_remove)
            child = self._children.pop(index)
            child.remove_all_children()
            self.num_children -= 1
        else:
            raise Exception('node is not a child for current Node')
        return self

    def remove_all_child(self):
        """Remove all children of this node."""
        self._children = []
        self.num_children = 0

    def remove_self(self):
        """Remove this node from its parents chilren list."""
        self.parent.remove_child(self)

    def get_parent(self):
        """Get the parent of this node."""
        return self.parent

    def generate_dot(self):
        """Generate dot code for this node and its connection to its children.

        Also recursively calls the children's generate_dot() functions
        """
        if self.parent is None:
            dot = 'digraph behavior_tree {\n ' \
                  'bgcolor="#F2F1EF" nodesep=.25 ranksep=.75 fixedsize=true;' \
                  'graph [fontname = "Courier-Bold" fontsize = 10 ' \
                  'fixedsize=true];\n' \
                  'node [fontname="Courier-Bold", fontsize=10 ' \
                  'fixedsize=true width=1.1 height=0.7];\n' \
                  'edge [arrowsize=0.4, arrowhead=vee];\n'
        else:
            dot = ''

        dot += self.get_style()

        for C in self._children:
            dot += C.generate_dot()
            dot += ' ' * 4 + self.name + ' ->' + C.name + ';\n'

        if self.parent is None:
            return dot + '}'
        else:
            return dot
        pass

    def get_style(self):
        """Get style of this node."""
        style = '    {} [shape={} style="rounded,filled", color={} '.format(
            self.name, self.shape, self.color)
        style += 'label="' + self.label + '\n' + \
            self.__class__.__name__ + '"];\n'

        return style

    def get_name(self):
        """Get name of this node."""
        return self.name

    def reset_(self):
        """Reset this node."""
        pass

    def reset(self):
        """Reset this node and its children."""
        self.reset_()
        # self.init()
        self.cycle = 0
        # self.color = LightGrey
        self.status = Status.FRESH
        for child in self._children:
            child.reset()

    def on_leave(self):
        """On leave."""
        pass

    def on_entry(self):
        """On entry."""
        pass

    def on_running(self):
        """On running."""
        pass

    def tick(self):
        """Virtual function that each node runs when getting a tick signal."""
        raise NotImplementedError


class Action(Node):
    """Action Node."""

    def __init__(self):
        """Init Action Node."""
        super(Action, self).__init__()
        self.label = '(Action)'
        self.shape = 'rectangle'
        # only Actions need behaviorRequest attr
        self.req = get_req()
        self.gc = get_gc()
        self.last_plat = VecPos(0, 0)

    def tick(self):
        """Tick Action Node."""
        raise NotImplementedError

    def add_child(self, child_to_add):
        """Cannot add child."""
        raise Exception('Action are leaf node, which should not have child')

    # methods for motion
    def walk(self, sx, sy, st):
        """Walk."""
        self.req.actions.body = walk(sx, sy, st)

    def kick(self, left=0):
        """Kick."""
        self.req.kick = True
        self.req.actions.body = kick(left)

    def update_vec(self, list_):
        """Update Gait Vec."""
        self.req.update_gait_vec = True
        self.req.gaitVec = list_to_gaitvec(list_)

    def enable_vec(self):
        """Enable Vec."""
        self.req.actions.body = wenxi_gaits()

    def goto(self, destination):
        """Goto position."""
        x, y, t = get_walk(destination, self.world.robot_pos)
        self.req.destination = destination
        # if x < 0:
        #     x = 0
        self.walk(x, y, t)

    def goto_field(self, field, angle):
        """Goto field."""
        x, y, t = get_walk_field(field, angle)
        self.walk(x, y, t)

    def got_dest(self, pos):
        """Check if getting to dest."""
        dis = get_dis(pos, self.world.robot_pos)
        diff_angle = abs_angle_diff(pos.anglez - self.world.robot_pos.anglez)

        if dis < 35 and diff_angle < 15:
            return True
        else:
            return False

    def enable_localization(self):
        """Enable Localization."""
        self.req.enable_localization = True

    def crouch(self):
        """Crouch."""
        self.req.actions.body = crouch()

    def stand(self):
        """Stand."""
        self.req.actions.body = standup()

    def step(self):
        """Step."""
        self.walk(0, 0, 0)

    def turn(self, st):
        """Turn."""
        self.req.actions.body = walk(0, 0, st)

    def turn_right(self):
        """Turn right."""
        self.turn(-6)

    def turn_left(self):
        """Turn left."""
        self.turn(6)

    def lookat(self, yaw, pitch, yaw_speed=2, pitch_speed=2):
        """Look at somewhere with given yaw and pitch."""
        self.req.actions.head = look_at(yaw, pitch, yaw_speed, pitch_speed)

    def face(self, field):
        """Face an object."""
        angle = get_angle(field)

        if angle < - 10:
            self.turn_right()
        elif angle > 10:
            self.turn_left()
        else:
            self.crouch()

    def gaze_at(self, position, speed=1):
        """Gaze at field position."""
        head = calc_center_plat(position)
        # print head, position
        self.lookat(head.x, head.y, speed, speed)

    def gaze_ball(self):
        """Gaze ball."""
        if self.world.see_ball:
            if get_magnitude(self.world.ball_field) < 25 and \
                abs(self.world.ball_field.y) < 25 and \
                    self.world.ball_field.x > 10:
                self.lookat(0, 60, 1, 1)
            else:
                self.gaze_at(self.world.ball_field)
            return True
        else:
            return False

    def plat_got_dest(self, plat):
        """Plat got dest."""
        return get_dis(self.world.plat, plat) < 0.01

    def set_position(self, robotstate):
        """Set self localization at given global position."""
        self.req.resetLocalization = True
        self.req.reset_point = robotstate

    def capture(self):
        """Capture images."""
        self.req.saveimage = True

    def debug_log(self):
        """Output debug log."""
        rospy.logdebug('{} ticking'.format(self.__class__.__name__))


class Condition(Node):
    """Condition Node."""

    def __init__(self):
        """Init Condition Node."""
        super(Condition, self).__init__()
        self.label = '(Condiction)'
        self.shape = 'oval'
        self.gc = get_gc()

    def tick(self):
        """Tick Condition Node."""
        raise NotImplementedError

    def add_child(self, child_to_add):
        """Cannnot add child."""
        raise Exception("Condiction is leaf node, which should't have child")
