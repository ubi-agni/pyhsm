import pyhsm_msgs.msg as msgs
import rospy
from std_msgs.msg import String

from .. import introspection
from . import GObject

__all__ = ["StateNode", "RootStateNode", "DummyStateNode"]


class StateNode(GObject.GObject):
    """A node representing a state in a HSM"""

    __slots__ = [slot for slot in msgs.HsmState.__slots__ if slot != "path"]

    def __init__(self, msg, root):
        """Initialize a state node from the given ``HsmState`` message

        :type msg: msgs.HsmState
        """
        GObject.GObject.__init__(self)
        assert isinstance(root, RootStateNode)
        self.root = root
        self._path = self.root.prefix + msg.path

        # store all slots from msg as instance properties
        for slot in self.__slots__:
            self.__setattr__(slot, getattr(msg, slot))

    # HSM-related methods

    @property
    def path(self):
        """Return the (full) path of this node, including a prefix from a root state"""
        return self._path

    def update(self, msg):
        """Update all msg slots and return True if there were any changes"""
        changed = False
        for slot in self.__slots__:
            value = getattr(msg, slot)
            if getattr(self, slot) != value:
                setattr(self, slot, value)
                changed = True
        return changed


class RootStateNode(StateNode):
    """A node representing the root state of a HSM."""

    def __init__(self, msg, prefix, server_name):
        """Initialize a state node from the given ``HsmState`` message and other meta information."""
        self._prefix = prefix
        # Ensure _prefix has a trailing slash
        if self._prefix and self._prefix[-1] != "/":
            self._prefix += "/"
        StateNode.__init__(self, msg, self)  # initialize after _prefix!
        self._server_name = server_name
        self._transition_publisher = rospy.Publisher(
            server_name + introspection.TRANSITION_TOPIC, String, queue_size=1
        )
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
        # Do NOT StateNode.__init__ to ensure that slots remain undefined
        self._path = path
        self.root = None
