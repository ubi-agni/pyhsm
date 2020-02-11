import pyhsm_msgs.msg as msgs

from gtk_wrap import Gtk, Pango
from state_node import DummyStateNode, RootStateNode, StateNode


class StateTreeModel(Gtk.TreeStore):
    """
    A tree model containing state nodes.

    Also provides a cell data function to render text from the internal nodes correctly
    (see ``render_path``).
    """

    STATE_COLUMN = 0
    """The column containing the state node."""

    def __init__(self):
        """Initialize an empty state tree model."""
        Gtk.TreeStore.__init__(self, StateNode)
        self._create_attr_methods()
        self._create_local_root_attr_methods()

        # Create root node
        self.append(None, DummyStateNode('/'))

    # Metaprogramming

    def _create_attr_methods(self):
        """
        Create methods propagating a state node property getter to the state node contained in a
        given node.
        """
        state_node_properties = ('path', 'parent_path', 'label',
                                 'initial_state', 'transitions', 'is_root_state')
        for attr in state_node_properties:
            attr_method = self._create_attr_method(attr)
            self.__setattr__(attr, attr_method)

    def _create_attr_method(self, attr):
        """
        Create a method propagating the given attribute getter to the state node contained in a
        given node.
        """

        def attr_method(self, node):
            # TODO Can we write a formatted doc string here? Would be sweet.
            return self.get_value(node).__getattr__(attr)

        return attr_method

    def _create_local_root_attr_methods(self):
        """
        Create methods that fetch the local root state node of a given node and propagate a property
        getter to it.
        """
        root_state_node_properties = ('prefix', 'server_name', 'publisher', 'active_state')
        for attr in root_state_node_properties:
            attr_method = self._create_local_root_attr_method(attr)
            self.__setattr__(attr, attr_method)

    def _create_local_root_attr_method(self, attr)
        """
        Create a method propagating the given attribute getter to the local root state node of the
        state node contained in a given node.
        """

        def attr_method(self, node):
            # TODO Can we write a formatted doc string here? Would be juicy.
            return self.local_root_state(node).__getattr__(attr)

        return attr_method

    # StateNode property functions

    # def path(self, node):
    #     """Return the state path stored in the given node."""
    #     return self.get_value(node).path

    # def parent_path(self, node):
    #     """Return the state parent path stored in the given node."""
    #     return self.get_value(node).parent_path

    # def label(self, node):
    #     """Return the state label stored in the given node."""
    #     return self.get_value(node).label

    # def initial_state(self, node):
    #     """Return the initial state stored in the given node."""
    #     return self.get_value(node).initial_state

    # def transitions(self, node):
    #     """Return the state transitions stored in the given node."""
    #     return self.get_value(node).transitions

    # RootStateNode property functions

    # def prefix(self, node):
    #     """Return the state prefix for the given node's HSM."""
    #     return self.local_root_state(node).prefix

    # def server_name(self, node):
    #     """Return the server name for the given node's HSM."""
    #     return self.local_root_state(node).server_name

    # def publisher(self, node):
    #     """Return the publisher for the given node's HSM."""
    #     return self.local_root_state(node).publisher

    # def active_state(self, node):
    #     """Return the active state for the given node's HSM."""
    #     return self.local_root_state(node).active_state

    # Tree convenience functions

    def full_path(self, node):
        """Return the full path of the given node. The full path includes the prefix."""
        path = self.path(node)
        prefix = self.prefix(node)
        if prefix:
            prefix = prefix + '/'
        return prefix + path

    def global_root_node(self):
        """Return the root node of the tree model or ``None`` if the tree is empty."""
        return self.get_iter_first()

    def global_root_state(self):
        """Return the root ``StateNode`` of the tree model or ``None`` if the tree is empty."""
        root_node = self.global_root_node
        return root_node and self.get_value(root_node)

    def local_root_node(self, node):
        """Return the root node of the given node's HSM in linear time."""
        while node is not None:
            if self.is_root_state(node):
                return node
            node = self.parent_node(node)
        raise ValueError('No root node that is a ``RootStateNode`` found.')

    def local_root_state(self, node):
        """Return the root ``StateNode`` of the given node's HSM in linear time."""
        return self.get_value(self.local_root_node(node))

    def parent_node(self, child):
        """Return the parent node of the given node or ``None`` if it has none."""
        return self.iter_parent(child)

    def parent_state(self, child):
        """Return the parent ``StateNode`` of the given node or ``None`` if it has none."""
        parent_node = self.parent_node(child)
        return parent_node and self.get_value(parent_node)

    def global_parent_nodes(self, child):
        """Return a generator over all sequential parent nodes of the given node."""
        parent = self.parent_node(child)
        while parent is not None:
            yield parent
            parent = self.parent_node(parent)

    def global_parent_states(self, child):
        """Return a generator over all sequential parent ``StateNode``s of the given node."""
        return map(self.get_value, self.global_parent_nodes(child))

    def local_parent_nodes(self, child):
        """Return a generator over all sequential parent nodes of the given node in its HSM."""
        for node in self.global_parent_nodes(child):
            yield node
            if self.is_root_state(node):
                return

    def local_parent_states(self, child):
        """
        Return a generator over all sequential parent ``StateNode``s of the given node in its HSM.
        """
        return map(self.get_value, self.local_parent_nodes(child))

    def child_nodes(self, parent):
        """Return a generator over the children of the given node."""
        child = self.iter_children(parent)
        while child is not None:
            yield child
            child = self.iter_next(child)

    def child_states(self, parent):
        """Return a generator over the child ``StateNode``s of the given node."""
        return map(self.get_value, self.child_nodes(parent))

    def find_node(self, path, return_last_match=False):
        """
        Return the node stored with the given path or ``None`` if it does not exist.
        If ``return_last_match`` is ``True``, the last match (or root if there was none)
        is returned.
        """
        parent = self.global_root_node()
        path_parts = path.split('/')
        # Strip an empty start (happens when the given path starts with '/').
        # We do this because we start by looking at the children of the root node.
        if path_parts and not path_parts[0]:
            path_parts = path_parts[1:]

        for i, path_part in enumerate(path_parts):
            for child in self.child_nodes(parent):
                label = self.label(child)

                if label == path_part:
                    # Label was a match
                    parent = child
                    break
                elif isinstance(child, DummyStateNode):
                    # We have a dummy state and need to cover the following cases:
                    #    1. Dummy path does not match label but matches whole path
                    #       (due to being longer).
                    #    2. Dummy path does not match label, does not match whole path but
                    #       matches part of whole path (due to being longer than label but
                    #       not containing the whole path).
                    #       For this, we need to correctly handle diving further down with the
                    #       path parts; we cannot just update the parent but need to match for
                    #       each path part until we see that we are not going to match with the
                    #       dummy node anymore. We then update the parent in hopes of finding a
                    #       child here that can continue following the path.
                    #    3. Dummy path does not match at all (we already cover this by
                    #       doing nothing).

                    child_path = self.path(child)
                    child_path_parts = child_path.split('/')

                    if child_path_parts[:len(path_parts)] == path_parts:
                        # Case 1
                        return child
                    elif child_path_parts[:i + 1] == path_parts[:i + 1]:
                        # Case 2

                        # This cannot result in an index error as we would have returned already
                        # with ``i + 1 == len(path_parts)`` and an equal path part comparison.
                        if child_path_parts[:i + 2] != path_parts[:i + 2]:
                            parent = child
                        break
                    # Case 3

            else:
                # No child with the correct path found
                if return_last_match:
                    return parent
                else:
                    return None
        return parent

    def find_state(self, path, return_last_match=False):
        """
        Return the ``StateNode`` stored with the given path or ``None`` if it does not exist.
        If ``return_last_match`` is ``True``, the last match (or the root state if there was none)
        is returned.
        """
        node = self.find_node(path, return_last_match)
        return node and self.get_value(node)

    def find_first_child_node(self, parent, pred):
        """
        Return the first child node under the given ``parent`` node where ``pred`` returns
        ``True`` or ``None`` if no match was found.
        ``pred`` is a 1-argument function each child node is passed to.
        """
        return next(n for n in self.child_nodes(parent) if pred(n), None)

    def find_first_child_state(self, parent, pred):
        """
        Return the first child ``StateNode`` under the given ``parent`` node where ``pred``
        returns ``True`` or ``None`` if no match was found.
        ``pred`` is a 1-argument function each child node is passed to.
        """
        child_node = self.find_first_child_node(parent, pred)
        return child_node and self.get_value(child_node)

    # Convenience overwrites

    def append(self, parent, row=None):
        """Append ``row`` to the ``parent`` node. If ``parent`` is ``None``, create a new root."""
        if row is not None:
            try:
                iter(row)
            except TypeError:
                row = (row,)
        Gtk.TreeStore.append(parent, row)

    def get_value(self, iter, column=self.STATE_COLUMN):
        """Return the value stored in the given tree ``iter`` at the given ``column``."""
        return Gtk.TreeStore.get_value(self, iter, column)

    # Dot code generation

    # FIXME (maybe new class?)

    # Structure message handling

    def build_from_structure_msg(self, msg, server_name, publisher, active_state=None):
        """
        Fill the ``StateTreeModel`` with the given ``HsmStructure`` message's contents and return
        the root node of the HSM.
        """
        if type(msg) is not msgs.HsmStructure:
            raise TypeError('``msg`` must be a ``HsmStructure`` message.')

        prefix_leaf = self._create_dummy_tree(msg.prefix)
        root_state = RootStateNode(msg.states[0], msg.prefix, server_name, publisher, active_state)
        local_root = self.append(prefix_leaf, root_state)
        parent = local_root

        for state_msg in msg.states[1:]:
            parent = self._add_node_from_state_msg(state_msg, parent, prefix_leaf)

        return local_root

    def _add_node_from_state_msg(self, state_msg, parent, excluded_upper_limit):
        """
        Add a new node under the given parent if there is no existing node at that location
        and return it (or the existing one).
        ``excluded_upper_limit`` is a node indicating at which point the parent hierarchy has
        been traveled too far upwards in search of a parent. It should be an ancestor of ``parent``.
        """
        path = state_msg.path

        # Search for an existing node with the given path
        existing_node = self.find_first_child_node(
            parent, lambda node: self.contains_path(node, path))
        if not existing_node:
            # TODO We could also update the existing node here instead of skipping.
            return existing_node

        state = StateNode(state_msg)

        if isinstance(existing_node, DummyStateNode):
            # We know the path exists in the tree but it is a dummy node.
            # First we get the correct dummy node we want to replace with our data...
            dummy_node = self._split_dummy(existing_node, path)

            # Then we replace the data...
            self.set_value(dummy_node, self.STATE_COLUMN, state)
            # And return the ex-dummy node as the new parent.
            return dummy_node
        else:
            # No special case; we can simply add a node to the tree under the correct parent.
            parent_path = state.parent_path

            while self.path(parent) != parent_path:
                # Parent is not correct; walk up until we get the correct one.
                parent = self.parent_node(parent)
                if parent == excluded_upper_limit:
                    raise ValueError(
                        'Could not get back to a valid parent during tree construction. '
                        "Searched for a parent with path '{}'.".format(parent_path))

            return self.append(parent, state)

    def _create_or_split_dummy(self, path):
        """
        Return a node for the given path. If the state with the given path is a dummy node for which
        the given path lies in its stored path, the dummy node is split at that location and the
        correct node returned.
        """
        node = self.find_node(path, return_last_match=True)

        if node and node != self.global_root_node():
            # We found a node under the given path.
            if self.path(node) == path:
                # It's a match!
                return node

            # We have a dummy that we need to split.
            return self._split_dummy(node, path)
        else:
            # We create the dummy from scratch.
            dummy_state = DummyStateNode(path)

            if isinstance(node, DummyStateNode):
                # If parent is a dummy, simply change its path instead.
                self.set_value(node, self.STATE_COLUMN, dummy_state)
                return node
            else:
                return self.append(node, dummy_state)

    def _split_dummy(self, node, path):
        """
        Return a node for the given path. If the given node contains a dummy node for which the
        given path lies in its stored path, the dummy node is split at that location and the
        correct node returned.
        """
        if self.path(node) == path:
            return node

        first_split = self.append(self.parent_node(node), DummyStateNode(path))
        # TODO Okay to not copy old value?
        second_split = self.append(first_split, self.get_value(node))

        # Sadly we have to 'move' the whole subtree by manually removing and adding it.
        # At least we do not have to care about existing nodes and other fun stuff.
        self._move_subtree(node, second_split)
        return first_split

    def _move_subtree(self, old_parent, new_parent):
        """Move the subtree under ``old_parent`` to ``new_parent`` and return ``new_parent``."""
        old_new_parents = [(old_parent, new_parent)]
        old_nodes = [old_parent]

        while parents:
            old_parent, new_parent = old_new_parents.pop()

            for old_child_node in self.children_nodes(old_parent):
                state = self.get_value(old_child_node)
                if self.path(new_parent) != state.parent_path:
                    raise ValueError(
                        'Did not get a valid parent node during dummy node splitting. '
                        "Wanted a parent with path '{}'.".format(state.parent_path))

                # TODO Okay to not copy old value?
                new_child_node = self.append(new_parent, state)

                old_nodes.append(old_child_node)
                old_new_parents.append((old_child_node, new_child_node))

        # Remove in reverse so we don't invalidate parents before their children
        for old_node in old_nodes[::-1]:
            self.remove(old_node)

        return new_parent

    def contains_path(self, node, path):
        """
        Find out whether the given node has the same path as the given path.
        Correctly handles dummy nodes.
        """
        node_path = self.path(node)
        if isinstance(node, DummyStateNode):
            path_parts = path.split('/')
            node_path_parts = node_path.split('/')
            return node_path_parts[:len(path_parts)] == path_parts
        else:
            return node_path == path

    @classmethod
    def from_structure_msg(cls, msg, server_name, publisher, active_state=None):
        """Return a new ``StateTreeModel`` from the given ``HsmStructure`` message's contents."""
        model = cls()
        model._build_from_structure_msg(msg, server_name, publisher, active_state)
        return model

    # State message handling

    def update_active_from_current_state_msg(self, msg):
        """
        Update the active state from the given ``HsmCurrentState`` message.
        If the state does not exist, do not do anything.
        """
        if type(msg) is not msgs.HsmCurrentState:
            raise TypeError('``msg`` must be a ``HsmCurrentState`` message.')

        active_path = msg.path
        active_node = self.find_node(active_path)
        if not active_node:
            return

        local_root_state = self.local_root_state(active_node)
        local_root_state.active_state = active_path

    # Static methods

    @staticmethod
    def render_path(column, cell, model, iter, user_data):
        """
        Cell data function to render the path of a state node in a tree view.
        Also applies a bold font weight if the state node is active and the renderer has
        ``weight_set`` set to ``True``.
        """
        state = model.get_value(iter)
        renderer.set_property('text', path)
        if renderer.weight_set and not isinstance(state, DummyStateNode):
            # TODO If this is too slow, create an extra column for the weight.
            active_state = model.active_state(iter)
            path = state_node.path
            # TODO Do we need an ``is`` instead of ``==`` here for speed?
            renderer.set_property('weight', model.create_weight(state == active_state))

    @staticmethod
    def create_weight(make_bold):
        """Return the correct ``Pango.Weight`` depending on the value of ``make_bold``."""
        if make_bold:
            return Pango.Weight.BOLD
        else:
            return Pango.Weight.NORMAL
