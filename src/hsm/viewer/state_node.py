import pyhsm_msgs.msg as msgs
import rospy
from std_msgs.msg import String

from .. import introspection
from gtk_wrap import GObject


class StateNode(GObject.GObject):
    """A node representing a state in a HSM"""

    def __init__(self, msg):
        """Initialize a state node from the given ``HsmState`` message

        :type msg: msgs.HsmState
        """
        GObject.GObject.__init__(self)
        self._msg = msg

    # HSM-related methods

    @property
    def path(self):
        """Return the path of this node. The path does not contain the prefix."""
        return self._msg.path

    @property
    def initial(self):
        """Return the label of the initial child state of this node."""
        return self._msg.initial

    @property
    def transitions(self):
        """Return a list of labels of other states that can be transitioned into from this node."""
        return self._msg.transitions


class RootStateNode(StateNode):
    """A node representing the root state of a HSM."""

    def __init__(self, msg, prefix, server_name):
        """Initialize a state node from the given ``HsmState`` message and other meta information."""
        StateNode.__init__(self, msg)
        self._prefix = prefix
        if not self._prefix or self._prefix[-1] != '/':
            self._prefix += '/'
        self._server_name = server_name
        self._transition_publisher = rospy.Publisher(server_name + introspection.TRANSITION_TOPIC, String, queue_size=1)
        self.current = None

    # HSM-related methods

    @property
    def prefix(self):
        return self._prefix

    @property
    def server_name(self):
        return self._server_name

    @property
    def transition_publisher(self):
        return self._transition_publisher


class DummyStateNode(StateNode):
    """A node representing an unknown path"""

    def __init__(self, path):
        """Initialize a dummy state node with the given path."""
        GObject.GObject.__init__(self)
        self._path = path

    @property
    def path(self):
        """Return the path of this node. The path does not contain the prefix."""
        return self._path

    @property
    def initial(self):
        raise AttributeError('DummyStateNodes do not have an initial state.')

    @property
    def transitions(self):
        raise AttributeError('DummyStateNodes do not have transitions.')
