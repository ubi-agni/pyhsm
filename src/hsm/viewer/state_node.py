import pyhsm_msgs.msg as msgs

from gtk_wrap import GObject


class StateNode(GObject.GObject):
    """
    A node mapping to a state in a HSM.
    The node is part of the tree of states representing a whole HSM.
    """

    def __init__(self, msg):
        """
        Initialize a state node from the given ``HsmState`` message as well as the other
        information designating a root and further structure to this message.

        Even though the parent does not have to be assigned, we throw an error if it is requested
        before being assigned.
        """
        GObject.GObject.__init__(self)
        if type(msg) is not msgs.HsmState:
            raise TypeError('``msg`` must be a ``HsmState`` message.')
        self._msg = msg

    # HSM-related methods

    @property
    def path(self):
        """Return the path of this node. The path does not contain the prefix."""
        return self._msg.path

    @property
    def parent_path(self):
        """Return the parent path of this node. The parent path is the path without the label."""
        return self.get_parent_path(self.path)

    @property
    def label(self):
        """Return the label of this node. The label is the part of the path after the last '/'."""
        return self.get_label(self.path)

    @property
    def initial_state(self):
        """
        Return the label of the initial child state of this node or ``None`` if this is a leaf.
        """
        return self._msg.initial

    @property
    def transitions(self):
        """Return a list of labels of other states that can be transitioned into from this node."""
        return self._msg.transitions

    # Tree- and hierarchy-related methods

    @property
    def is_root_state(self):
        """
        Return whether this node is a root node of a HSM.
        May have a parent node related to a different HSM.
        """
        return isinstance(self, RootStateNode)

    # Static methods

    @staticmethod
    def get_label(path):
        """
        Return the label part of the given path.
        The label is the part of the path after the last '/'.
        """
        last_slash_index = path.rfind('/')
        return path[last_slash_index + 1:]

    @staticmethod
    def get_parent_path(path):
        """
        Return the parent part of the given path (the path without the label).
        If the path has no parent, return the empty string. If ``path`` is empty, return ``None``.
        """
        if not path:
            return None
        last_slash_index = path.rfind('/')
        if last_slash_index < 0:
            return ''
        return path[:last_slash_index]


class RootStateNode(StateNode):
    """
    A node mapping to the root or top level state in a HSM.
    This is the root node of the tree of states representing a whole HSM.
    """

    def __init__(self, msg, prefix, server_name, publisher, active_state=None):
        """
        Initialize a state node from the given ``HsmState`` message as well as other information
        related to the server of this HSM.
        """
        StateNode.__init__(self, msg)
        self._prefix = prefix
        self._server_name = server_name
        self._publisher = publisher
        self._active_state = active_state

    # HSM-related methods

    @property
    def prefix(self):
        return self._prefix

    @property
    def server_name(self):
        return self._server_name

    @property
    def publisher(self):
        return self._publisher

    @property
    def active_state(self):
        """
        Return the currently active state in the HSM associated with this tree or ``None`` if
        no state has been marked as active yet.
        """
        return self._active_state

    @active_state.setter
    def active_state(self, active_state):
        """Set the currently active state to the given one."""
        # TODO Check for whether the state is in this tree maybe?
        if not isinstance(active_state, StateNode):
            raise TypeError('``active_state`` must be a ``StateNode``.')
        self._active_state = active_state


class DummyStateNode(StateNode):
    """
    A node containing only a path. Used to create prefix paths that have not received their
    corresponding structure.
    """

    def __init__(self, path):
        """Initialize a dummy state node with the given path."""
        # TODO store visible path as well (the new parts)
        GObject.GObject.__init__(self)
        self._path = path

    @property
    def path(self):
        """Return the path of this node. The path does not contain the prefix."""
        return self._path

    @property
    def initial_state(self):
        """
        Return the label of the initial child state of this node or ``None`` if this is a leaf.
        """
        raise AttributeError('``DummyStateNode``s do not have an initial state.')

    @property
    def transitions(self):
        """Return a list of labels of other states that can be transitioned into from this node."""
        raise AttributeError('``DummyStateNode``s do not have transitions.')
