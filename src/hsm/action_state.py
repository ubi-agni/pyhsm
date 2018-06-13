import hsm
import rospy
import threading
import logging
import copy
import sys
import actionlib

__all__ = ['ActionState']
_LOGGER = logging.getLogger("hsm.action_state")

class ActionState(hsm.Container):
    """Simple action client state.

    Use this class to represent an actionlib as a state in a state machine.
    """

    # Meta-states for this action
    WAITING_FOR_SERVER = 0
    PENDING = 1 # WAITING_FOR_SERVER + have pending action goal
    INACTIVE = 2
    ACTIVE = 3
    EXITING = 4

    def __init__(self, client=None, action_name=None, action_spec=None,
                 goal = None, name=None, server_wait_timeout = 10.0):
        """Constructor for ActionState action client wrapper.

        @type client: actionlib.SimpleActionClient
        @param client: SimpleActionClient instance to use.

        @type action_name: string
        @param action_name: The name of the action as it will be broadcast over ros.

        @type action_spec: actionlib action msg
        @param action_spec: The type of action to which this client will connect.

        @type goal: actionlib goal msg
        @param goal: If the goal for this action does not need to be generated at
        runtime, it can be passed to this state on construction.

        @type server_wait_timeout: C{rospy.Duration}
        @param server_wait_timeout: This is the timeout used for aborting while
        waiting for an action server to become active.
        """
        if client is not None and (action_name is not None or action_spec is not None):
            raise ValueError('Cannot handle both, client and action arguments')

        if isinstance(client, actionlib.SimpleActionClient):
            self._action_client = client
        else:
            self._action_client = actionlib.SimpleActionClient(action_name, action_spec)
        self._action_name = self._action_client .action_client.ns

        # Initialize base class
        super(ActionState, self).__init__(name if name is not None else self._action_name)

        self.goal = goal if goal is not None else copy.copy(self._action_client.action_client.ActionGoal())

        self._status = ActionState.WAITING_FOR_SERVER
        self._server_wait_timeout = rospy.Duration(server_wait_timeout)

        # Wait for action client to become active
        self._action_wait_thread = threading.Thread(name=self._action_name + '/wait_for_server', target=self._wait_for_server)
        self._action_wait_thread.start()

        self.add_handler('enter',self._on_enter)
        self.add_handler('exit', self._on_exit)

    def _wait_for_server(self):
        """Internal method for waiting for the action server
        This is run in a separate thread and allows construction of this state
        to not block the construction of other states.
        """
        timeout_time = rospy.get_rostime() +  self._server_wait_timeout
        while (self._status == ActionState.WAITING_FOR_SERVER or self._status == ActionState.PENDING) \
                and not rospy.is_shutdown() and rospy.get_rostime() < timeout_time:
            try:
                if self._action_client.wait_for_server(rospy.Duration(0.1)):
                    self._on_server_ready()
                    return
            except:
                print("Unexpected error:", sys.exc_info()[0])
                if not rospy.core._in_shutdown: # wait_for_server should not throw an exception just because shutdown was called
                    break
        _LOGGER.warning("Action server '{0}' not found for {1:.1f} seconds."
                        .format(self._action_name, (rospy.get_rostime() - timeout_time + self._server_wait_timeout).to_sec()))
        self._on_server_failed()

    def _on_server_ready(self):
        if self._status == ActionState.PENDING:
            self._status = ActionState.INACTIVE
            self._send_goal()
        else:
            self._status = ActionState.INACTIVE

    def _on_server_failed(self):
        if self._status == ActionState.EXITING or self._status == ActionState.WAITING_FOR_SERVER:
            return
        self._status = ActionState.WAITING_FOR_SERVER
        self.root.dispatch('TIMEOUT')


    def _on_enter(self, state, event):
        _LOGGER.debug("status: %d" % self._status)
        # If we immediately re-entered the state after exiting, status might be still EXITING
        if self._status == ActionState.EXITING:
            if self._action_wait_thread.is_alive():
                # If _action_wait_thread still active, simply continue it
                self._status = ActionState.WAITING_FOR_SERVER
            else:
                self._status = ActionState.INACTIVE

        if self._status == ActionState.WAITING_FOR_SERVER:
            self._status = ActionState.PENDING
            if not self._action_wait_thread.is_alive():
                self._action_wait_thread = threading.Thread(name=self._action_name + '/wait_for_server',
                                                            target=self._wait_for_server)
                self._action_wait_thread.start()

        elif self._status == ActionState.INACTIVE:
            self._send_goal()
        else:
            raise Exception('unexpected internal state %d' % self._status)

    def _on_exit(self, state, event):
        if self._status == ActionState.ACTIVE:
            _LOGGER.debug("Cancel goal request on '{}'".format(self._action_name))
            self._action_client.cancel_goal()
        self._status = ActionState.EXITING

    def _send_goal(self):
        # Dispatch goal via non-blocking call to action client
        self._status = ActionState.ACTIVE
        self._action_client.send_goal(self.goal, self._goal_done_cb, self._goal_active_cb, self.feedback_cb)

    ### Action client callbacks
    def _goal_active_cb(self):
        """Goal Active Callback
        This callback starts the timer that watches for the timeout specified for this action.
        """
        self._status = ActionState.ACTIVE

    def feedback_cb(self, feedback):
        pass

    def _goal_done_cb(self, result_state, result):
        """Goal Done Callback
        This callback resets the active flags and reports the duration of the action.
        Also, if the user has defined a result_cb, it is called here before the
        method returns.
        """
        def get_result_str(i):
            strs = ('PENDING','ACTIVE','PREEMPTED','SUCCEEDED','ABORTED','REJECTED','LOST')
            if i < len(strs):
                return strs[i]
            else:
                return 'UNKNOWN ('+str(i)+')'

        cancelled = (self._status == ActionState.EXITING)
        self._status = ActionState.INACTIVE
        if cancelled:
            return

        _LOGGER.debug("Action {} sent result {}({})".format(self._action_name, get_result_str(result_state), result_state))
        self.root.dispatch(hsm.core.Event(get_result_str(result_state), result=result))
