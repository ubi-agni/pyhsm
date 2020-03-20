from __future__ import print_function

import rospy
import pyhsm_msgs.msg as msgs

from gtk_wrap import Gtk, Pango
from state_node import DummyStateNode, RootStateNode, StateNode


class StateTreeModel(Gtk.TreeStore):
    """
    Tree model representing the state hierarchy.

    We distinguish tree items (``Gtk.TreeIter`` instances) and state nodes.
    """
    # column indexes of the model
    STATE = 0     # StateNode
    PATH = 1      # full path
    LABEL = 2     # short name shown
    WEIGHT = 3    # font weight, indicating active state
    ENABLED = 4   # is item enabled for selection?

    def __init__(self):
        """Initialize an empty state tree model."""
        Gtk.TreeStore.__init__(self, StateNode, str, str, int, bool)

    def row(self, item):
        """Retrieve all row information for given tree item"""
        return [self.get_value(item, i) for i in range(5)]

    def append(self, parent, state, weight=Pango.Weight.NORMAL):
        # Determine (short) display label
        if parent is not None:
            parent_path = self.state(parent).path
            path = state.path
            assert(path.startswith(parent_path + '/'))
            # label is determined by stripping of parent's path from state.path
            label = path[len(parent_path)+1:]
        else:
            label = state.path

        enabled = not isinstance(state, DummyStateNode)

        return Gtk.TreeStore.append(self, parent, [state, state.path, label, weight, enabled])

    def path(self, item):
        return self.get_value(item, column=self.PATH)

    def label(self, item):
        return self.get_value(item, column=self.LABEL)

    def state(self, item):
        """Retrieve ``StateNode`` of the given tree item"""
        return self.get_value(item, column=self.STATE)

    def root_state(self, item):
        """Return the ``RootStateNode`` of the given tree item."""
        return self.get_value(item, column=self.STATE).root

    def children(self, parent=None):
        item = self.iter_children(parent)
        while item is not None:
            yield item
            item = self.iter_next(item)

    def can_mount_at(self, item):
        """Can we mount a HSM root node at item?"""
        if not isinstance(self.state(item), DummyStateNode) and self.get_value(item, self.ENABLED) == False:
            return True  # outdated (disabled) normal node
        return self.iter_children(item) is None  # leaf node

    def find_node(self, path, parent=None, first=None, max_depth=-1):
        """Return the next tree item matching the given path.

        If parent is given, *start* search with first child of parent.
        If first is given, *continue* search with next child after first
        """
        def _find(item, path, depth):
            """Find tree item with given path, starting from item."""
            while item is not None:
                label = self.label(item)
                if path.startswith(label):  # potential match
                    tail = path[len(label):]
                    if not tail:  # full match: done
                        return item
                    if tail[0] == '/':  # partial match
                        # continue on deeper level
                        path = tail.lstrip('/')  # remove leading slashes
                        if not path:
                            return item
                        elif depth != 0:
                            depth -= 1
                            item = self.iter_children(item)
                            continue
                # spurious or no match, continue on current level
                item = self.iter_next(item)

        path = path.lstrip('/')  # remove leading slashes

        if first is None:  # start search from parent
            if not path: return parent  # special case: path already empty -> can return parent
            first = self.iter_children(parent)
        elif parent is not None:
            raise ValueError('Arguments parent and first are mutually exclusive.')
        else:  # continue search from first
            parent = self.iter_parent(first)
            parent_path = parent and self.path(parent) or ''
            path = path[len(parent_path):]  # strip path from first
            first = self.iter_next(first)

        return _find(first, path, max_depth)

    # Dot code generation

    # FIXME Base this on old ContainerNode dot code generation code (maybe new class?).

    # Structure message handling

    def update_tree(self, msg, server_name):
        """Update the model with the content from the ``HsmStructure`` message

        :type msg: msgs.HsmStructure
        :returns the root item of the HSM
        """
        if not isinstance(msg, msgs.HsmStructure):
            raise TypeError('``msg`` must be a ``HsmStructure`` message.')

        parent = self._create_or_split_dummy(msg.prefix)  # create dummy parent node

        parent = root = self._create_root_state(parent, msg.states[0], msg.prefix, server_name)

        for state_msg in msg.states[1:]:
            parent = self._create_state(state_msg, parent, root)

        return root

    def _create_root_state(self, parent, msg, prefix, server_name):
        """Create or update a RootStateNode below parent according to msg"""
        root = self.find_node(msg.path, parent=parent, max_depth=0)
        while True:
            if root is None:
                root_state = RootStateNode(msg, prefix, server_name)
                root = self.append(parent, state=root_state)
            else:
                root_state = self.state(root)
                if isinstance(root_state, RootStateNode):
                    if root_state.server_name == server_name:
                        self._update_state(root_state, msg)
                    elif parent is None:  # continue searching, eventually insert additional root state at top level
                        root = self.find_node(msg.path, first=root, max_depth=0)
                        continue
                    else:  # otherwise fail
                        raise RuntimeError('server_name has changed: {} -> {}'.format(root_state.server_name, server_name))
                else:
                    if self.can_mount_at(root):
                        raise RuntimeError('Failed to insert HSM root {} at state {}'.format(msg.path, root_state.path))
                    # remove all children from root (in case of invalidated item)
                    for item in self.children(root):
                        self.remove(item)

                    # update model with new RootStateNode
                    root_state = RootStateNode(msg, prefix, server_name)
                    self.set(root, self.STATE, root_state, self.ENABLED, True)
            return root

    def _create_state(self, msg, parent, root):
        """Create or update a normal StateNode

        :arg possible_parent:
        """
        # find actual parent, by traversing the tree upwards (from parent to root) until path matches
        path = self.state(root).prefix + msg.path
        parent_path = self.path(parent) + '/'
        while not path.startswith(parent_path):
            if parent is root:
                raise ValueError('Failed to find valid parent for state {}'.format(path))
            parent = self.iter_parent(parent)
            parent_path = self.path(parent) + '/'

        tail = path[len(parent_path):]
        existing = self.find_node(tail, parent, max_depth=0)

        if existing is None:
            rospy.logdebug('CONSTRUCTING: ' + path)
            return self.append(parent, StateNode(msg, self.root_state(root)))
        else:
            state = self.state(existing)  # turn tree item into associated state
            self._update_state(state, msg)
            # TODO Update existing node
            return existing

    def _update_state(self, state, msg):
         assert isinstance(state, StateNode)

    def _create_or_split_dummy(self, path):
        """Return a (dummy) node for the given path.

        If required, a dummy node will be split to reach the desired path.
        """
        if not path:
            return None

        def _find(item, path):
            """Find tree item with given path, starting from item."""
            while item is not None:
                label = self.label(item)
                if path.startswith(label):  # potential match
                    tail = path[len(label):]
                    if tail[0] == '/':  # partial match
                        tail = tail.lstrip('/')  # remove leading slashes
                        if not tail:  # nothing left -> full match: done
                            return item
                        # continue on deeper level
                        return _find(self.iter_children(item), tail) \
                               or self.append(item, DummyStateNode(self.state(item).path + '/' + tail))
                elif isinstance(self.state(item), DummyStateNode) and label.startswith(path + '/'):
                    # match within dummy node: split and return
                    return self._split_dummy(item, path)

                item = self.iter_next(item)

        item = _find(self.iter_children(None), path)
        if item is None:  # create new dummy at root level
            return self.append(None, DummyStateNode(path))
        else:
            return item

    def _split_dummy(self, item, path):
        """Split a dummy item and return the tree item corresponding to path.

        Given a dummy item with label a/b/c and path a/b, create new dummy items a/b and c.
        """
        state = self.state(item)
        assert(isinstance(state, DummyStateNode))
        assert(state.path.startswith(path + '/'))

        parent = self.append(self.iter_parent(item), DummyStateNode(path))
        child = self.append(parent, DummyStateNode(state.path))

        # Move the subtree below item by manually removing and adding it corresponding items.
        self._move_subtree(item, child)
        self.swap(item, parent)
        self.remove(item)  # remove old item (and all its children)
        return parent

    def _move_subtree(self, old_parent, new_parent):
        """Move the subtree under ``old_parent`` to ``new_parent`` and return ``new_parent``."""
        old_new_parents = [(old_parent, new_parent)]
        while old_new_parents:
            old_parent, new_parent = old_new_parents.pop()

            for old_child in self.children(old_parent):
                # move item with all its row info
                new_child = Gtk.TreeStore.append(self, new_parent, self.row(old_child))
                # schedule pair for processing the children as well
                old_new_parents.append((old_child, new_child))

        return new_parent

    def enable(self, item, value, recursive=False):
        self.set_value(item, self.ENABLED, value)
        if recursive:
            for child in self.children(item):
                self.enable(child, value, recursive)

    def update_current_state(self, msg, root_state):
        """Update the active state from the given ``HsmCurrentState`` message

        :return whether the active state has changed
        """
        root = self.find_node(root_state.path)
        while root and self.state(root) is not root_state:
            root = self.find_node(root_state.path, first=root)
        if root is None:
            rospy.logerr('Invalidated root state "{}" ({}) for state update.'.
                         format(root_state.path, root_state.server_name))
            return False

        strip = len(root_state.path)
        old_current = root_state.current and \
                      self.find_node(root_state.current.path[strip:], parent=root)
        new_current = self.find_node((root_state.prefix + msg.path)[strip:], parent=root)
        changed = new_current != old_current
        root_state.current = None if new_current is None else self.state(new_current)

        if changed:
            if new_current is not None:
                self.set_value(new_current, self.WEIGHT, Pango.Weight.BOLD)
            if old_current is not None:
                self.set_value(old_current, self.WEIGHT, Pango.Weight.NORMAL)

        return changed
