import pyhsm_msgs.msg as msgs

from hsm.core import _History, any_event

# TODO We could refactor so these methods are again part of ContainerProxy;
# in the end, we could build the hierarchy from the proxies instead
# of from the machine (and if the proxy is root, publish the structure msg).
#
# The above is all a big maybe; maybe this structure is more desirable.

# Magic word for transitions to previous state
HISTORY_TRANSITION_MAGIC_WORD = '__HISTORY'

# Types for the public API
STATUS_MSG = msgs.HsmCurrentState
STRUCTURE_MSG = msgs.HsmStructure

# What type to filter ROS topics by to get these messages
STATUS_MSG_TOPIC_TYPE = 'pyhsm_msgs/HsmCurrentState'


def build_status_msg(path):
    return msgs.HsmCurrentState(path)


def build_structure_msg(prefix, machine):
    """Return a structure message with the given prefix for the
    given state machine.
    """
    states = _build_state_msgs(machine)

    # Construct structure message
    structure_msg = msgs.HsmStructure(prefix, states)
    return structure_msg


def _build_state_msgs(machine):
    """Walk through the given state machine and build messages from
    all states.

    This method is expected to iterate in pre-order so the tree can later be
    created by reversing the returned list.
    """
    parent_hierarchy = []
    node_stack = [machine]
    state_msgs = []

    while node_stack:
        node = node_stack.pop()
        # If the node is not a leaf ``State``
        if hasattr(node, 'states'):
            node_stack.extend(node.states)

        # Update path correctly; handle changing parent paths
        _update_parent_hierarchy_(parent_hierarchy, node)
        curr_path = '/'.join(map(lambda s: s.name, parent_hierarchy))

        state_msg = _build_state_msg(node, curr_path)
        state_msgs.append(state_msg)

    return state_msgs


def _update_parent_hierarchy_(parent_hierarchy, node):
    """Update the parent hierarchy in place according to the newly
    visited node.
    """
    if not parent_hierarchy or node.parent == parent_hierarchy[-1]:
        # If current node is child of previously visited node or if it
        # is the first (root) node, add it to the parent hierarchy.
        parent_hierarchy.append(node)
    else:
        # Otherwise, go up the chain until we find the direct parent...
        while node.parent != parent_hierarchy[-1]:
            del parent_hierarchy[-1]
            # ... and add the current node to the hierarchy.
        parent_hierarchy.append(node)
    return parent_hierarchy


def _build_state_msg(state, full_path):
    """Return the state messages for the given state with the given full
    path up to and including it.
    """
    # If state is not a leaf ``State``
    try:
        initial_state = state.initial_state.name
    except AttributeError:
        initial_state = ''
    transitions = _build_transition_msgs(state)

    state_msg = msgs.HsmState(full_path, initial_state, transitions)
    return state_msg


def _build_transition_msgs(state):
    """Build transition messages for all transitions of the given state."""
    transition_msgs = []
    transition_to_events_dict = {}

    # Set up transition -> events mapping
    for event, transitions in state._transitions.items():
        if event is any_event:
            event = '*'
        for transition in transitions:
            # Transitions are ``dict``s, those cannot be hashed
            hashable_transition = frozenset(transition.items())
            if hashable_transition not in transition_to_events_dict:
                transition_to_events_dict[hashable_transition] = [event]
            else:
                transition_to_events_dict[hashable_transition].append(event)

    for transition, events in transition_to_events_dict.items():
        transition = dict(transition)
        # Handle HISTORY transitions specially
        target = transition['to_state']
        if isinstance(target, _History):
            target = HISTORY_TRANSITION_MAGIC_WORD
        else:
            target = target.name

        transition_msg = msgs.HsmTransition(
            events=events,
            target=target,
            # Convert these functions to strings
            condition=transition['condition'].__name__,
            action=transition['action'].__name__,
            before=transition['before'].__name__,
            after=transition['after'].__name__,
        )
        transition_msgs.append(transition_msg)

    return transition_msgs
