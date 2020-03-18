import rospy
import rostopic
from std_msgs.msg import String

import hsm
from pyhsm_msgs.msg import HsmStructure, HsmState, HsmCurrentState, HsmTransition

__all__ = ['IntrospectionClient', 'IntrospectionServer']

# Topic names
STATUS_TOPIC = '/current_state'
STRUCTURE_TOPIC = '/structure'
TRANSITION_TOPIC = '/transition'
EVENT_TOPIC = '/event'

# Magic word for transitions to history state
HISTORY_TRANSITION_MAGIC_WORD = '__HISTORY'


class IntrospectionClient():
    def get_servers(self):
        """Get the base names that are broadcasting HSM structures."""
        topics = rostopic.find_by_type(HsmStructure._type)
        return [t[:t.rfind(STRUCTURE_TOPIC)] for t in topics]


class IntrospectionServer():
    """Server for providing introspection and control for an HSM."""

    def __init__(self, server_name, machine, prefix):
        self._server_name = server_name
        self._machine = machine
        self._prefix = prefix
        self._current = None

        # Advertise structure publisher
        self._structure_pub = rospy.Publisher(
            name=server_name + STRUCTURE_TOPIC,
            data_class=HsmStructure,
            queue_size=1,
            latch=True)

        self._status_pub = rospy.Publisher(
            name=server_name + STATUS_TOPIC,
            data_class=HsmCurrentState,
            queue_size=1,
            latch=True)

    def start(self):
        self._publish_structure()
        # publish transitions
        self._machine.register_transition_cb(self._transition_cb)
        self._transition_cb(None, self._machine.leaf_state)

        # subscribe to transition and event triggers
        self._transition_sub = rospy.Subscriber(
            self._server_name + TRANSITION_TOPIC,
            String, self._transition_cmd_cb)
        self._event_sub = rospy.Subscriber(
            self._server_name + EVENT_TOPIC,
            String, self._event_trigger_cb)

    def stop(self):
        del self._transition_sub
        del self._event_sub
        self._machine.unregister_transition_cb(self._transition_cb)

    def _transition_cb(self, from_state, to_state):
        self._current = self._get_full_path(to_state)
        self._publish_status()

    @staticmethod
    def _get_full_path(state):
        """Return the full path up to the given state."""
        names = []
        while state is not None:
            names.append(state.name)
            state = state.parent

        # As we walked up from a child, we need to reverse the path's parts.
        names.reverse()
        return '/'.join(names)

    def _transition_cmd_cb(self, msg):
        to_state = self._machine
        try:
            path = msg.data.split('/')
            if to_state.name != path[0]:
                raise IndexError('Invalid root state name')
            for k in path[1:]:
                to_state = to_state[k]
        except IndexError:
            rospy.logerr('Unknown state: {}'.format(msg.data))
            return

        # dispatch an event to trigger the transition
        # (don't call _transition_to() directly!)
        self._machine.dispatch(hsm.Event('__TRANSITION__', to_state=to_state))

    def _event_trigger_cb(self, msg):
        self._machine.dispatch(msg.data)

    def _state_msgs(self):
        """Traverse the state tree of the machine and collect messages for all states."""
        parent_hierarchy = []
        node_stack = [self._machine]
        state_msgs = []

        while node_stack:
            node = node_stack.pop()
            if hasattr(node, 'states'):
                node_stack.extend(node.states)  # schedule all children

            # Update path correctly; handle changing parent paths
            self._update_parent_hierarchy_(parent_hierarchy, node)
            path = '/'.join(map(lambda s: s.name, parent_hierarchy))

            state_msg = self._state_msg(node, path)
            state_msgs.append(state_msg)

        return state_msgs

    @staticmethod
    def _update_parent_hierarchy_(parent_hierarchy, node):
        """Update the parent hierarchy in place according to the newly visited node."""
        if not parent_hierarchy or node.parent is parent_hierarchy[-1]:
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

    @staticmethod
    def _state_msg(state, full_path):
        """Return the state messages for the given state with the given full
        path up to and including it.
        """
        # If state is not a leaf ``State``
        try:
            initial_state = state.initial_state.name
        except AttributeError:
            initial_state = ''
        transitions = IntrospectionServer._transition_msgs(state)

        return HsmState(full_path, initial_state, transitions)

    @staticmethod
    def _transition_msgs(state):
        """Build transition messages for all transitions of the given state."""
        transition_msgs = []
        transition_to_events_dict = {}

        # Set up transition -> events mapping
        for event, transitions in state._transitions.items():
            if event is hsm.core.any_event:
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
            if isinstance(target, hsm.core._History):
                target = HISTORY_TRANSITION_MAGIC_WORD
            else:
                target = target.name

            transition_msg = HsmTransition(
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

    def _publish_structure(self):
        try:
            self._structure_pub.publish(HsmStructure(self._prefix, self._state_msgs()))
        except Exception:
            if not rospy.is_shutdown():
                rospy.logerr("Publishing HSM structure message failed.")

    def _publish_status(self):
        """Publish current state of this container."""
        self._status_pub.publish(HsmCurrentState(self._current))
