import pickle
import threading

import rospy
import rostopic
import smach
from smach_msgs.msg import SmachContainerInitialStatusCmd
from std_msgs.msg import String

import hsm
from hsm.core import Container
from hsm import msg_builder

__all__ = ['IntrospectionClient', 'IntrospectionServer']

# Topic names
STATUS_TOPIC = '/current_state'
INIT_TOPIC = '/smach/container_init'
STRUCTURE_TOPIC = '/structure'
TRANSITION_TOPIC = '/transition'
EVENT_TOPIC = '/event'


class IntrospectionClient():
    def get_servers(self):
        """Get the base names that are broadcasting smach states."""

        # Get the currently broadcasted smach introspection topics
        topics = rostopic.find_by_type(msg_builder.STATUS_MSG_TOPIC_TYPE)

        return [t[:t.rfind(STATUS_TOPIC)] for t in topics]

    def set_initial_state(self,
                          server,
                          path,
                          initial_states,
                          timeout=None):
        """Set the initial state of a smach server.

        @type server: string
        @param server: The name of the introspection server to which this
        client should connect.

        @type path: string
        @param path: The path to the target container in the state machine.

        @type initial_states: list of string
        @param inital_state: The state the target container should take when it
        starts. This is as list of at least one state label.

        @type timeout: rospy.Duration
        @param timeout: Timeout for this call. If this is set to None, it will
        not block, and the initial state may not be set before the target state
        machine goes active.
        """

        # Construct initial state command
        initial_status_msg = SmachContainerInitialStatusCmd(
            path=path,
            initial_states=initial_states,
            local_data=pickle.dumps(smach.UserData(), 2))

        # A status message to receive confirmation that the state was
        # set properly
        msg_response = msg_builder.STATUS_MSG()

        # Define a local callback to just stuff a local message
        def local_cb(msg, msg_response):
            rospy.logdebug("Received status response: " + str(msg))
            msg_response.path = msg.path

        # Create a subscriber to verify the request went through
        state_sub = rospy.Subscriber(server + STATUS_TOPIC,
                                     msg_builder.STATUS_MSG,
                                     callback=local_cb,
                                     callback_args=msg_response)

        # Create a publisher to send the command
        rospy.logdebug("Sending initial state command: "
                       + str(initial_status_msg.path) + " on topic '"
                       + server + INIT_TOPIC + "'")
        init_pub = rospy.Publisher(server + INIT_TOPIC,
                                   SmachContainerInitialStatusCmd,
                                   queue_size=1)
        init_pub.publish(initial_status_msg)

        start_time = rospy.Time.now()

        # Block until we get a new state back
        if timeout is not None:
            while rospy.Time.now() - start_time < timeout:
                # Send the initial state command
                init_pub.publish(initial_status_msg)

                # Filter messages that are from other containers
                if msg_response.path == path:
                    # Check if the heartbeat came back to match
                    # FIXME TODO
                    # state_match = all([s in msg_response.initial_states
                    #                    for s in initial_states])
                    state_match = True

                    rospy.logdebug("STATE MATCH: " + str(state_match))

                    if state_match:
                        return True
                rospy.sleep(0.3)
            return False


class ContainerProxy():
    """Smach Container Introspection proxy.

    This class is used as a container for introspection and debugging.
    """

    def __init__(self, server_name, container, path,
                 update_rate=rospy.Duration(2.0)):
        """Constructor for tree-wide data structure."""
        self._path = path
        self._container = container
        # self._update_rate = update_rate
        # self._status_pub_lock = threading.Lock()

        # Advertise init service
        # TODO most likely not necessary anymore
        # self._init_cmd = rospy.Subscriber(
        #     server_name + INIT_TOPIC,
        #     SmachContainerInitialStatusCmd,
        #     self._init_cmd_cb)

        self._keep_running = False

    def start(self):
        self._keep_running = True
        # self._status_pub_thread.start()

    def stop(self):
        self._keep_running = False

    def _status_pub_loop(self):
        """Loop to publish the status heartbeat."""
        while not rospy.is_shutdown() and self._keep_running:
            # TODO
            self._publish_status('HEARTBEAT')
            try:
                end_time = rospy.Time.now() + self._update_rate
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    rospy.sleep(0.1)
            except:
                pass

    def _publish_status(self, info_str=""):
        """Publish current state of this container."""
        # Construct messages
        with self._status_pub_lock:
            path = self._path

            # Construct status message
            state_msg = msg_builder.build_status_msg(path)
            # Publish message
            self._status_pub.publish(state_msg)

    def _init_cmd_cb(self, msg):
        """Initialize a tree's state and userdata."""
        initial_states = msg.initial_states
        local_data = msg.local_data

        # Check if this init message is directed at this path
        rospy.logdebug('Received init message for path: ' + msg.path
                       + ' to ' + str(initial_states))
        if msg.path == self._path:
            if all(s in self._container.get_children()
                   for s in initial_states):
                ud = smach.UserData()
                ud._data = pickle.loads(msg.local_data)
                rospy.logdebug("Setting initial state in smach path: '"
                               + self._path + "' to '" + str(initial_states)
                               + "' with userdata: " + str(ud._data))

                # Set the initial state
                self._container.set_initial_state(
                    initial_states,
                    ud)
                # Publish initial state
                self._publish_status("REMOTE_INIT")
            else:
                rospy.logerr("Attempting to set initial state in container '"
                             + self._path + "' to '")
                # + str(initial_states)
                # + "', but this container only has states: "
                # + str(self._container.get_children()))


class IntrospectionServer():
    """Server for providing introspection and control for smach."""

    def __init__(self, server_name, machine, path,
                 update_rate=rospy.Duration(2.0)):
        """Traverse the smach tree starting at root, and construct
        introspection proxies for getting and setting debug state.
        """

        # A list of introspection proxies
        self._proxies = []

        # Store args
        self._server_name = server_name
        self._machine = machine
        self._path = path
        self._active_path = ''

        self._status_pub_lock = threading.Lock()
        self._update_rate = update_rate

        # Advertise structure publisher
        self._structure_pub = rospy.Publisher(
            name=server_name + STRUCTURE_TOPIC,
            data_class=msg_builder.STRUCTURE_MSG,
            queue_size=1,
            latch=True)

        self._status_pub = rospy.Publisher(
            name=server_name + STATUS_TOPIC,
            data_class=msg_builder.STATUS_MSG,
            queue_size=1,
            latch=True)

        # TODO While the function is not reworked, we can comment this out.
        # self._structure_pub_thread = threading.Thread(
        #     name=server_name + ':structure_publisher',
        #     target=self._structure_pub_loop)

        self._keep_running = False

    def start(self):
        # Construct proxies
        proxy = self.construct(self._server_name, self._machine, self._path)
        # get informed about transitions
        self._machine.register_transition_cb(self._transition_cb)

        self._keep_running = True

        # TODO after rework, uncomment and
        #      maybe remove `_publish_structure` line
        # self._structure_pub_thread.start()
        self._publish_structure()
        self._publish_status()

        self._transition_cmd = rospy.Subscriber(
            self._server_name + TRANSITION_TOPIC,
            String, self._transition_cmd_cb)
        self._transition_cmd = rospy.Subscriber(
            self._server_name + EVENT_TOPIC,
            String, self._event_trigger_cb)

    def stop(self):
        self._keep_running = False
        for proxy in self._proxies:
            proxy.stop()

    def construct(self, server_name, state, path):
        """Recursively construct proxies to containers."""
        if path == '/':
            path = ''
        elif path and path[-1] != '/':
            path = path + '/'

        path = path + state.name

        # Construct a new proxy
        proxy = ContainerProxy(server_name, state, path)

        # Update active state
        if state.is_active():
            self._active_path = path

        # Get a list of children that are also containers
        for child in state.states:

            # If this is also a container, recurse into it
            if isinstance(child, Container):
                self.construct(server_name, child, path)
            elif child.is_active():
                # In case it is _not_ a container, we still want to update
                # the active state
                self._active_path = path + '/' + child.name

        # Store the proxy
        self._proxies.append(proxy)
        return proxy

    def clear(self):
        """Clear all proxies in this server."""
        self._proxies = []

    def _transition_cb(self, from_state, to_state):
        self._active_path = self._get_full_path(to_state)
        self._publish_status()

    def _get_full_path(self, state):
        """Return the full path up to the given state."""
        names = []
        while state is not None:
            names.append(state.name)
            state = state.parent
        if self._path:
            names.append(self._path)

        # As we walked up from a child, we need to reverse the path's parts.
        names.reverse()
        return '/'.join(names)

    def _transition_cmd_cb(self, msg):
        to_state = self._machine
        prefix = self._path + '/' if self._path else ''
        prefix += to_state.name
        if not msg.data.startswith(prefix):
            rospy.logerr('Invalid path prefix. Expecting: ' + prefix)
            return
        path = msg.data[len(prefix)+1:]

        for k in path.split('/'):
            to_state = to_state[k]
            if to_state is None:
                rospy.logerr('Unknown state {}'.format(msg.data))
                return

        # dispatch an event to trigger the transition
        # (don't call _transition_to() directly!)
        self._machine.dispatch(hsm.Event('__TRANSITION__', to_state=to_state))

    def _event_trigger_cb(self, msg):
        self._machine.dispatch(msg.data)

    # TODO Rework this function so it only publishes if the HSM
    # structure has changed.
    def _structure_pub_loop(self):
        """Loop to publish the status and structure heartbeats."""
        while not rospy.is_shutdown() and self._keep_running:
            self._publish_structure('CHANGE')
            try:
                end_time = rospy.Time.now() + self._update_rate
                while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                    rospy.sleep(0.1)
            except:
                pass

    def _publish_structure(self):
        path = self._path

        structure_msg = msg_builder.build_structure_msg(path, self._machine)
        try:
            self._structure_pub.publish(structure_msg)
        except:
            if not rospy.is_shutdown():
                rospy.logerr(
                    "Publishing SMACH introspection structure message failed.")

    def _publish_status(self):
        """Publish current state of this container."""
        # Construct messages
        with self._status_pub_lock:
            path = self._active_path

            # Construct status message
            state_msg = msg_builder.build_status_msg(path)
            # Publish message
            self._status_pub.publish(state_msg)
