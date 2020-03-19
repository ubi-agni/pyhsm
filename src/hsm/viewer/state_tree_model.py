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
    ROOT = 1      # associated RootStateNode
    LABEL = 2     # short name shown
    WEIGHT = 3    # font weight, indicating active state
    ENABLED = 4   # is item enabled for selection?

    def __init__(self):
        """Initialize an empty state tree model."""
        Gtk.TreeStore.__init__(self, StateNode, RootStateNode, str, int, bool)

    def row(self, item):
        """Retrieve all row information for given tree item"""
        return [self.get_value(item, i) for i in range(5)]

    def append(self, parent, state, root=None, label=None, weight=Pango.Weight.NORMAL):
        if root is None:
            if parent is not None:
                root = self.root_state(parent)
            elif not isinstance(state, DummyStateNode):
                raise ValueError('undefined root')

        if label is None:
            if parent is not None:
                parent_path = self.state(parent).path
                path = state.path
                assert(path.startswith(parent_path + '/'))
                label = path[len(parent_path)+1:]
            elif isinstance(state, DummyStateNode):
                label = state.path
            else:
                raise ValueError('undefined label')

        enabled = not isinstance(state, DummyStateNode)

        return Gtk.TreeStore.append(self, parent, [state, root, label, weight, enabled])

    def label(self, item):
        return self.get_value(item, column=self.LABEL)

    def state(self, item):
        """Retrieve ``StateNode`` of the given ``TreeIter``"""
        return self.get_value(item, column=self.STATE)

    def root_state(self, item):
        """Return the ``RootStateNode`` of the given ``TreeIter``."""
        return self.get_value(item, column=self.ROOT)

    def is_root_state(self, item):
        return self.get_value(item, column=self.ROOT) is self.get_value(item, column=self.STATE)

    def full_path(self, item, path=None):
        """Return the full path of the given item, including the prefix."""
        if path is None:
            path = self.state(item).path
        return self.root_state(item).prefix + path

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

    def find_node(self, path, parent=None, max_depth=-1):
        """Return the tree item matching given path, starting at parent item."""
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

        if parent is None:
            path = path.lstrip('/')  # remove leading slashes
        return _find(self.iter_children(parent), path, max_depth)

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
        if root is None:
            root_state = RootStateNode(msg, prefix, server_name)
            root = self.append(parent, state=root_state, root=root_state, label=msg.path)
        else:
            root_state = self.state(root)
            if isinstance(root_state, RootStateNode):
                if root_state.server_name != server_name:
                    raise RuntimeError('server_name has changed: {} -> {}'.format(root_state.server_name, server_name))
            else:
                if self.can_mount_at(root):
                    raise RuntimeError('Failed to insert HSM root {} at state {}'.format(msg.path, self.full_path(root)))
                # remove all children from root (in case of invalidated item)
                for item in self.children(root):
                    self.remove(item)

                # create new RootStateNode and update
                root_state = RootStateNode(msg, prefix, server_name)
                self.set_value(root, self.STATE, root_state)
                self.set_value(root, self.ROOT, root_state)
        return root

    def _create_state(self, msg, parent, root):
        """Create or update a normal StateNode

        :arg possible_parent:
        """
        # find actual parent, by traversing the tree upwards (from parent to root) until full_path matches
        full_path = self.full_path(root, msg.path)
        while not full_path.startswith(self.full_path(parent) + '/'):
            if parent is root:
                raise ValueError('Failed to find valid parent for state {}'.format(full_path))
            parent = self.iter_parent(parent)

        tail = full_path[len(self.full_path(parent))+1:]
        existing = self.find_node(tail, parent, max_depth=0)

        if existing is None:
            rospy.logdebug('CONSTRUCTING: ' + full_path)
            return self.append(parent, StateNode(msg))
        else:
            assert isinstance(existing, StateNode)
            # TODO Update existing node
            return existing

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

    def update_current_state(self, msg, root_state):
        """Update the active state from the given ``HsmCurrentState`` message

        :return whether the active state has changed
        """
        root_path = root_state.prefix + root_state.path
        root = self.find_node(root_path)
        assert self.state(root) is root_state

        if root_state.current is not None:
            old_current = self.find_node(root_state.prefix + root_state.current.path)
        else:
            old_current = None

        new_current = self.find_node(msg.path, self.iter_parent(root))
        changed = new_current != old_current
        root_state.current = None if new_current is None else self.state(new_current)

        if changed:
            if new_current is not None:
                self.set_value(new_current, self.WEIGHT, Pango.Weight.BOLD)
            if old_current is not None:
                self.set_value(old_current, self.WEIGHT, Pango.Weight.NORMAL)

        return changed
