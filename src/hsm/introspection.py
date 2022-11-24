import rospy
import rostopic
import hsm

from std_msgs.msg import String
from pyhsm_msgs.msg import HsmStructure, HsmState, HsmCurrentState, HsmTransition

__all__ = [
    "IntrospectionClient",
    "IntrospectionServer",
    "STATUS_TOPIC",
    "STRUCTURE_TOPIC",
    "TRANSITION_TOPIC",
    "EVENT_TOPIC",
]

# Topic names
STATUS_TOPIC = "/current_state"
STRUCTURE_TOPIC = "/structure"
TRANSITION_TOPIC = "/transition"
EVENT_TOPIC = "/event"

# Magic word for transitions to history state
HISTORY_TRANSITION_MAGIC_WORD = "__HISTORY"


class IntrospectionClient(object):
    @staticmethod
    def get_servers():
        """Get the base names that are broadcasting HSM structures."""
        topics = rostopic.find_by_type(HsmStructure._type)
        return [t[: t.rfind(STRUCTURE_TOPIC)] for t in topics]

    @staticmethod
    def subscribe(server_name, callback, queue_size=50, **kwargs):
        """Subscribe to a new structure messaging server."""
        return rospy.Subscriber(
            server_name + STRUCTURE_TOPIC,
            HsmStructure,
            callback=callback,
            queue_size=queue_size,
            **kwargs
        )


class IntrospectionServer(object):
    """Server for providing introspection and control for an HSM."""

    def __init__(self, server_name, machine, prefix):
        self._server_name = server_name
        self._machine = machine
        self._prefix = prefix
        self._current = None

        # Before ROS shutdown, call self.stop()
        rospy.on_shutdown(self.stop)

        # Advertise structure publisher
        self._structure_pub = rospy.Publisher(
            name=server_name + STRUCTURE_TOPIC, data_class=HsmStructure, queue_size=1, latch=True
        )

        self._status_pub = rospy.Publisher(
            name=server_name + STATUS_TOPIC, data_class=HsmCurrentState, queue_size=1, latch=True
        )

    def start(self):
        self.publish_structure()
        # publish transitions
        self._machine.register_transition_cb(self._transition_cb)
        self._transition_cb(None, self._machine.leaf_state)

        # subscribe to transition and event triggers
        self._transition_sub = rospy.Subscriber(
            self._server_name + TRANSITION_TOPIC, String, self._transition_cmd_cb
        )
        self._event_sub = rospy.Subscriber(
            self._server_name + EVENT_TOPIC, String, self._event_trigger_cb
        )

    def stop(self):
        try:
            del self._transition_sub
            del self._event_sub
        except AttributeError:
            return  # not started (or stopped)
        self._machine.unregister_transition_cb(self._transition_cb)
        self.publish_structure([])  # unregister from viewer

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
        return "/".join(names)

    def _transition_cmd_cb(self, msg):
        to_state = self._machine
        try:
            path = msg.data.split("/")
            if to_state.name != path[0]:
                raise IndexError("Invalid root state name")
            for k in path[1:]:
                to_state = to_state[k]
        except IndexError:
            rospy.logerr("Unknown state: {}".format(msg.data))
            return

        # dispatch an event to trigger the transition
        # (don't call _transition_to() directly!)
        self._machine.dispatch(hsm.Event("__TRANSITION__", to_state=to_state))

    def _event_trigger_cb(self, msg):
        self._machine.dispatch(msg.data)

    @staticmethod
    def _state_msgs(machine):
        """Traverse the state tree of the machine and collect messages for all states."""
        parent_hierarchy = []
        node_stack = [machine]
        state_msgs = []

        while node_stack:
            node = node_stack.pop()  # fetch nodes from the end of the stack
            if hasattr(node, "states"):
                # Schedule children in reverse order, to have correct parent_hierarchy!
                node_stack.extend(reversed(node.states))

            # Remove nodes from the tail of the list as long as they are not the node's parent
            while parent_hierarchy and (node.parent is not parent_hierarchy[-1]):
                del parent_hierarchy[-1]
            # Finally, add the new node
            parent_hierarchy.append(node)

            path = "/".join(map(lambda s: s.name, parent_hierarchy))

            state_msg = IntrospectionServer._state_msg(node, path)
            state_msgs.append(state_msg)

        return state_msgs

    @staticmethod
    def _state_msg(state, full_path):
        """Return the state messages for the given state with the given full
        path up to and including it.
        """
        # If state is not a leaf ``State``
        try:
            initial_state = state.initial_state.name
        except AttributeError:
            initial_state = ""
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
                event = "*"
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
            target = transition["to_state"]
            if isinstance(target, hsm.core._History):
                target = (
                    IntrospectionServer._get_full_path(target.parent) + HISTORY_TRANSITION_MAGIC_WORD
                )
            else:
                target = IntrospectionServer._get_full_path(target)

            transition_msg = HsmTransition(
                events=events,
                target=target,
                # Convert these functions to strings
                condition=transition["condition"].__name__,
                action=transition["action"].__name__,
                before=transition["before"].__name__,
                after=transition["after"].__name__,
            )
            transition_msgs.append(transition_msg)

        return transition_msgs

    def publish_structure(self, states=None):
        try:
            if states is None:
                states = self._state_msgs(self._machine)
            self._structure_pub.publish(HsmStructure(self._prefix, states))
        except Exception as e:
            if not rospy.is_shutdown():
                rospy.logerr("Publishing HSM structure message failed:\n" + str(e))

    def _publish_status(self):
        """Publish current state of this container."""
        self._status_pub.publish(HsmCurrentState(self._current))
