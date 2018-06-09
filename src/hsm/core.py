"""

hsm.core
--------

This module provides the core elements for hierarchical state machines: Event, State, StateMachine

--------
.. |StateMachine| replace:: :class:`~.StateMachine`
.. |State| replace:: :class:`~.State`
.. |Hashable| replace:: :class:`~collections.Hashable`
.. |Iterable| replace:: :class:`~collections.Iterable`
.. |Callable| replace:: :class:`~collections.Callable`
"""
# for Python 2 / 3 code compatibility
from __future__ import print_function
from six import iteritems

import collections
from collections import deque
from threading import Thread
import Queue

import logging
_LOGGER = logging.getLogger("hsm.core")


string_types = (str, unicode)
any_event = object()


def listify(obj):
    """Wraps a passed object into a list in case it has not been a list, tuple before.
    Returns an empty list in case ``obj`` is None.
    Args:
        obj: instance to be converted into a list.
    Returns:
        list: May also return a tuple in case ``obj`` has been a tuple before.
    """
    if obj is None:
        return []
    return obj if isinstance(obj, (list, tuple)) else [obj]


def _call(handler, *args, **kwargs):
    """Call handlers, handling a single function or a list of functions"""
    for cb in listify(handler):
        cb(*args, **kwargs)


class StateMachineException(Exception):
    """All |StateMachine| exceptions are of this type."""
    pass


class Event(object):
    """Events trigger actions and transitions.

    An event has a name and can carry userdata.
    The following attributes are set after an event has been dispatched:

    **Attributes:**

        .. attribute:: _machine

            The |StateMachine| instance that is handling the event.

        .. attribute:: propagate

            An event is propagated from the current state up the state hierarchy
            until it encounters a handler that can handle the event.
            To propagate it further (after first handling), set this flag to True in a handler.

    :param name: Name of an event. It may be anything as long as it's hashable.
    :type name: |Hashable|
    :param \*\*userdata: All keyword arguments are passed as userdata to handlers.

    **Example Usage:**

    .. code-block:: python

        state_machine.dispatch(Event('start'))
        state_machine.dispatch(Event('start', key='value'))
    """
    def __init__(self, name, **userdata):
        self.name = name
        self.propagate = True
        self.userdata = userdata
        self._machine = None

    def __repr__(self):
        return '<Event {0}, userdata={1}>'.format(self.name, self.userdata)


class TransitionsContainer(collections.defaultdict):
    def __init__(self):
        super(TransitionsContainer, self).__init__(list)

    def add(self, event, transition):
        self[event].append(transition)

    def get(self, event, from_state):
        for key in [event.name, any_event]:
            for transition in self[key]:
                if transition['condition'](from_state, event) is True:
                    return transition
        return None


class _History(object):
    def __init__(self, parent):
        self.parent = parent


class State(object):
    """Represents a state in a state machine.

    `enter` and `exit` handlers are called whenever a state is entered or
    exited respectively. These action names are reserved only for this purpose.

    It is encouraged to extend this class to encapsulate a state behavior,
    similarly to the State Pattern.

    **Attributes:**

        .. attribute:: state
            Current, local state (instance of |State|) in a state machine.

        .. attribute:: leaf_state
            See the :attr:`~.StateMachine.leaf_state` property.

        **root**
            See the :attr:`~.StateMachine.root` property.

    :param name: Human readable state name
    :type name: str

    **Example Usage:**

    .. code-block:: python

        # Extending State to encapsulate state-related behavior.
        class Running(State):
            def on_enter(self, state, event):
                print('Running state entered')

            def on_jump(self, state, event):
                print('Jumping')

    .. code-block:: python

        # An alternative option to add handlers is by calling :func:`add_handler`.
        # A handler may be any function as long as it takes `state` and `event` args.
        def my_handler(state, event):
            print('my handler')

        running = State('running')
        running.add_handler('event', my_handler)

    """
    def __init__(self, name):
        self.name = name
        self.parent = None
        self._handlers = {}
        self._transitions = TransitionsContainer()

        # register handlers for methods with name "on_*"
        for trigger, value in iteritems(self.__class__.__dict__):
            if trigger.startswith('on_') and callable(value):
                self.add_handler(trigger[3:], value)

    def add_handler(self, trigger, func):
        """ Add a new event callback.

        :param trigger: name of triggering event
        :type trigger: str
        :param func: callback function
        :type func: callable
        """
        handlers = self._handlers.get(trigger, [])
        handlers.append(func)
        if len(handlers) == 1:
            self._handlers[trigger] = handlers

    def add_transition(self, events, target_state, condition=None, action=None, before=None, after=None):
        """Add a transition from self to target_state

        All callbacks take two arguments - `state` and `event`. See parameters
        description for details.

        It is possible to create conditional if/elif/else-like logic for
        transitions. To do so, add many same transition rules with different
        condition callbacks. First met condition will trigger a transition, if
        no condition is met, no transition is performed.

        :param target_state: Target state. If `None`, then it's an `internal
            transition <https://en.wikipedia.org/wiki/UML_state_machine
            #Internal_transitions>`_
        :type target_state: |State|, `None`
        :param events: List of events that trigger the transition
        :type events: |Iterable| of |Hashable|
        :param condition: Condition callback - if returns `True` transition may
            be initiated.

            `condition` callback takes two arguments:

                - state: Leaf state before transition
                - event: Event that triggered the transition

        :type condition: |Callable|
        :param action: Action callback that is called during the transition
            after all states have been left but before the new one is entered.

            `action` callback takes two arguments:

                - state: Leaf state before transition
                - event: Event that triggered the transition

        :type action: |Callable|
        :param before: Action callback that is called right before the
            transition.

            `before` callback takes two arguments:

                - state: Leaf state before transition
                - event: Event that triggered the transition

        :type before: |Callable|
        :param after: Action callback that is called just after the transition

            `after` callback takes two arguments:

                - state: Leaf state after transition
                - event: Event that triggered the transition

        :type after: |Callable|

        """
        # Rather than adding some if statements later on, let's just declare some
        # neutral items that will do nothing if called. It simplifies the logic a lot.
        if action is None:
            action = self._nop
        if before is None:
            before = self._nop
        if after is None:
            after = self._nop
        if condition is None:
            condition = self._nop

        events = listify(events)
        Validator(self).validate_add_transition(self, target_state, events)

        transition = {
            'to_state': target_state,
            'condition': condition,
            'action': action,
            'before': before,
            'after': after,
        }
        for event in events:
            self._transitions.add(event, transition)

    def __repr__(self):
        return '<State {0} ({1})>'.format(self.name, hex(id(self)))

    @property
    def root(self):
        """Get the root state in a states hierarchy.

        :returns: Root state in the states hierarchy
        :rtype: |State|
        """
        while self.parent is not None:
            self = self.parent
        return self

    def is_substate(self, state):
        """Check whether the `state` is a substate of `self`.

        Also `self` is considered a substate of `self`.

        :param state: State to verify
        :type state: |State|
        :returns: `True` if `state` is a substate of `self`, `False` otherwise
        :rtype: bool
        """
        parent = self
        while parent:
            if parent is state:
                return True
            parent = parent.parent
        return False

    def is_active(self):
        # state is active if its the root, or if its parent's state is self
        return self.parent is None or self.parent.state is self

    def _on(self, event):
        try:
            handler = self._handlers[event.name]
            event.propagate = False
            _call(handler, self, event)
        except KeyError:
            pass  # event not handled in this state

        # Never propagate exit/enter events, even if propagate is set to True
        if (self.parent and event.propagate and
                event.name not in ['exit', 'enter']):
            self.parent._on(event)

    def _nop(self, state, event):
        return True


class Container(State):
    """Container interface.

    Containers allow for hierarchical nesting of states.
    """

    def __init__(self, name):
        super(Container, self).__init__(name)
        self.states = set()
        # current local state
        self.state = None
        # previous local state (for transitions to history)
        self._prev_state = None
        # special history state
        self.HISTORY = _History(self)

    def __getitem__(self, key):
        if isinstance(key, State):
            return key

        def find_by_name(name):
            for state in self.states:
                if state.name == name:
                    return state
            return None

        keys = key.split('.', 1)
        state = find_by_name(keys[0])
        return state if len(keys) == 1 else state[keys[1]]

    def add_state(self, state, initial=False):
        """Add a state to a state the container.

        If states are added, one (and only one) of them has to be declared as
        `initial`.

        :param state: State to be added. It may be an another |Container|
        :type state: |State|
        :param initial: Declare a state as initial
        :type initial: bool
        """
        if isinstance(state, string_types):
            state = State(state)
        Validator(self).validate_add_state(state, initial)
        state.initial = initial
        state.parent = self
        self.states.add(state)
        return state

    def add_states(self, *states):
        """Add multiple `states` to the Container.

        :param states: A list of states to be added
        """
        for state in states:
            self.add_state(state)

    @property
    def initial_state(self):
        """Get the initial state in a state machine.

        :returns: Initial state in a state machine
        :rtype: |State|

        """
        for state in self.states:
            if state.initial:
                return state
        return None

    def set_initial_state(self, state):
        """Set an initial state in a state machine.

        :param state: Set this state as initial in a state machine
        :type state: |State|
        """
        Validator(self).validate_set_initial(state)
        state.initial = True

    def add_transition(self, events, target_state, *args, **kwargs):
        # handle string names: retrieve State instances
        super(Container, self).add_transition(events, self[target_state], *args, **kwargs)

    @property
    def leaf_state(self):
        """Get the current leaf state.

        The :attr:`~.StateMachine.state` property gives the current,
        local state in a state machine. The `leaf_state` goes to the bottom in
        a hierarchy of states. In most cases, this is the property that should
        be used to get the current state in a state machine, even in a flat
        FSM, to keep the consistency in the code and to avoid confusion.

        :returns: Leaf state in a hierarchical state machine
        :rtype: |State|

        """
        return self._get_leaf_state(self)

    def _get_leaf_state(self, state):
        while hasattr(state, 'state') and state.state is not None:
            state = state.state
        return state

    @property
    def history_state(self):
        """Get the history state of this |Container|.

        :returns: Leaf state in a hierarchical state machine
        :rtype: |State|
        """
        result = self
        while hasattr(result, '_prev_state') and result._prev_state is not None:
            result = result._prev_state
        return result


class StateMachine(Container):
    """State machine controls actions and transitions.

    To provide the State Pattern-like behavior, the formal state machine rules
    may be slightly broken, and instead of creating an `internal transition
    <https://en.wikipedia.org/wiki/UML_state_machine #Internal_transitions>`_
    for every action that doesn't require a state change, event handlers may be
    added to states. These are handled first when an event occurs. After that
    the actual transition is called, calling `enter`/`exit` actions and other
    transition actions. Nevertheless, internal transitions are also supported.

    So the order of calls on an event is as follows:

        1. State's event handler
        2. `condition` callback
        3. `before` callback
        4. `exit` handlers
        5. `action` callback
        6. `enter` handlers
        7. `after` callback

    If there's no handler in states or transition for an event, it is silently ignored.

    If using nested state machines, all events should be sent to the root state machine.

    .. note ::

        |StateMachine| extends |State| and therefore it is possible to always
        use a |StateMachine| instance instead of the |State|. This wouldn't
        be a good practice though, as the |State| class is designed to be as
        small as possible memory-wise and thus it's more memory efficient. It
        is valid to replace a |State| with a |StateMachine| later on if there's
        a need to extend a state with internal states.

    .. note::

        For the sake of speed thread safety isn't guaranteed.

    **Example Usage:**

    .. code-block:: python

        state_machine = StateMachine('root')
        state_on = State('On')
        state_off = State('Off')
        state_machine.add_state('Off', initial=True)
        state_machine.add_state('On')
        state_machine.add_transition(state_on, state_off, events=['off'])
        state_machine.add_transition(state_off, state_on, events=['on'])
        state_machine.initialize()
        state_machine.dispatch(Event('on'))
    """

    def __init__(self, name):
        super(StateMachine, self).__init__(name)
        self._transition_cbs = []

    def _get_transition(self, event, leaf_state):
        state = leaf_state
        while state is not None:
            transition = state._transitions.get(event, state)
            if transition is not None:
                return transition
            state = state.parent
        return None

    def initialize(self):
        """Initialize states in the state machine.

        After a state machine has been created and all states are added to it,
        :func:`initialize` has to be called.

        If using nested state machines (HSM),
        :func:`initialize` has to be called on a root
        state machine in the hierarchy.

        """
        states = deque()
        states.append(self)
        validator = Validator(self)
        while states:
            state = states.popleft()
            validator.validate_initial_state(state)
            if state.is_active():
                state.state = state.initial_state

            for child_state in state.states:
                if isinstance(child_state, Container):
                    states.append(child_state)
        self._enter_states(None, None, self.state)


    def register_transition_cb(self, transition_cb, *args):
        """Adds a transition callback to this container."""
        self._transition_cbs.append((transition_cb, args))

    def call_transition_cbs(self):
        """Calls the registered transition callbacks.
        Callback functions are called with two arguments in addition to any
        user-supplied arguments:
         - userdata
         - a list of active states
         """
        for (cb, args) in self._transition_cbs:
            cb(*args)

    def dispatch(self, event):
        """Dispatch an event to a state machine.

        If using nested state machines (HSM), it has to be called on a root
        state machine in the hierarchy.

        :param event: Event to be dispatched
        :type event: :class:`.Event`

        """
        if isinstance(event, string_types):
            event = Event(event)
        event._machine = self
        leaf_state_before = self.leaf_state

        # TODO: _on(event) handler and registered transitions should be mutually exclusive
        # leaf_state_before._on(event)

        transition = self._get_transition(event, leaf_state_before)
        if transition is None:
            return
        to_state = transition['to_state']
        if isinstance(to_state, _History):
            to_state = to_state.parent.history_state

        transition['before'](leaf_state_before, event)
        top_state = self._exit_states(event, leaf_state_before, to_state)
        transition['action'](leaf_state_before, event)
        self._enter_states(event, top_state, to_state)
        transition['after'](self.leaf_state, event)
        self.call_transition_cbs()

    def _transition_to(self, to_state):
        """manual transition to given state"""
        top_state = self._exit_states(None, self.leaf_state, to_state)
        self._enter_states(None, top_state, to_state)
        self.call_transition_cbs()

    def _exit_states(self, event, from_state, to_state):
        if to_state is None:
            return None
        state = self.leaf_state
        while (state.parent and
                not (from_state.is_substate(state) and
                     to_state.is_substate(state)) or
                (state == from_state == to_state)):
            _LOGGER.debug('exiting %s', state.name)
            exit_event = Event('exit', propagate=False, source_event=event)
            exit_event._machine = self
            try: # TODO: really ignore all exceptions?
                state._on(exit_event)
            except: # ignore exceptions due to event being None
                pass
            state.parent._prev_state = state
            state.parent.state = None
            state = state.parent
        return state

    def _enter_states(self, event, top_state, to_state):
        if to_state is None:
            return
        path = []
        state = self._get_leaf_state(to_state)

        while state.parent and state != top_state:
            path.append(state)
            state = state.parent
        if top_state is None:
            path.append(self)

        for state in reversed(path):
            _LOGGER.debug('entering %s', state.name)
            enter_event = Event('enter', propagate=False, source_event=event)
            enter_event._machine = self
            try: # TODO: really ignore all exceptions?
                state._on(enter_event)
            except: # ignore exceptions due to event being None
                pass
            if state.parent is not None:
                state.parent.state = state

    def get_active_states(self):
        return [self.state.name]


class Validator(object):
    def __init__(self, state_machine):
        self._machine = state_machine
        self.template = 'Machine "{0}" error: {1}'.format(
            self._machine.name, '{0}')

    def _raise(self, msg):
        raise StateMachineException(self.template.format(msg))

    def validate_add_state(self, state, initial):
        if not isinstance(state, State):
            msg = 'Unable to add state of type {0}'.format(type(state))
            self._raise(msg)
        self._validate_state_already_added(state)
        if initial is True:
            self.validate_set_initial(state)

    def _validate_state_already_added(self, state):
        root_machine = self._machine.root
        machines = deque()
        machines.append(root_machine)
        while machines:
            machine = machines.popleft()
            if state in machine.states and machine is not self._machine:
                msg = ('Machine "{0}" error: State "{1}" is already added '
                       'to machine "{2}"'.format(
                           self._machine.name, state.name, machine.name))
                self._raise(msg)
            for child_state in machine.states:
                if isinstance(child_state, StateMachine):
                    machines.append(child_state)

    def validate_set_initial(self, state):
        for added_state in self._machine.states:
            if added_state.initial is True and added_state is not state:
                msg = ('Unable to set initial state to "{0}". '
                       'Initial state is already set to "{1}"'
                       .format(state.name, added_state.name))
                self._raise(msg)

    def validate_add_transition(self, from_state, to_state, events):
        self._validate_from_state(from_state)
        self._validate_to_state(to_state)
        self._validate_events(events)

    def _validate_from_state(self, from_state):
        if not isinstance(from_state, State):
            msg = 'Unable to add transition from unknown state "{0}"'.format(from_state.name)
            self._raise(msg)

    def _validate_to_state(self, to_state):
        root_machine = self._machine.root
        if to_state is None:
            return
        elif to_state is root_machine:
            return
        elif isinstance(to_state, _History):
            to_state = to_state.parent  # check for existence of parent

        if not to_state.is_substate(root_machine):
            msg = 'Unable to add transition to unknown state "{0}"'.format(to_state.name)
            self._raise(msg)

    def _validate_events(self, events):
        if not isinstance(events, collections.Iterable):
            msg = ('Unable to add transition, events is not iterable: {0}'
                   .format(events))
            self._raise(msg)

    def validate_initial_state(self, machine):
        if machine.states and not machine.initial_state:
            msg = 'Machine "{0}" has no initial state'.format(machine.name)
            self._raise(msg)


class ProcessorThread(Thread):
    def __init__(self, state, event):
        super(ProcessorThread, self).__init__()
        self.state = state
        self.event = event
        self.interrupted = False

    def run(self):
        result = self.state.execute(self.event)
        if not self.interrupted:
            self.state.root.dispatch(Event(result))


class ProcessingState(State):
    def __init__(self, name):
        """State executing a procedure in a background thread

        The result of execute() method is triggered as a new event in the state machine.
        The state can be left at any time during execution of the procedure (due to an event triggering a transition).
        In this case the cancel flag is set to True, indicating that the Thread's result is not needed anymore.
        """
        super(ProcessingState, self).__init__(name)
        self._thread = None
        self.preempt_requested = False
        self.add_handler('enter',self._on_enter)
        self.add_handler('exit', self._on_exit)

    def execute(self, event):
        """Method executed while this state is active

        The return value of this function triggers a corresponding event.
        """
        raise NotImplementedError()

    def request_preempt(self):
        self.preempt_requested = True

    def _on_enter(self, state, event):
        self.preempt_requested = False
        self._thread = ProcessorThread(self, event)
        self._thread.start()

    def _on_exit(self, state, event):
        # indicate that the thread should cancel
        self._thread.interrupted = True
        self.request_preempt()


class CallbackState(ProcessingState):
    def __init__(self, name, callback, *args, **kwargs):
        """State executing a given function while being active."""
        super(CallbackState,self).__init__(name)
        self._cb = callback
        self._args = args
        self._kwargs = kwargs

    def execute(self, event = None):
        return self._cb(event, *self._args, **self._kwargs)


def run(sm, final_state):
    """Run the statemachine until the final state is reached. Events are dispatched a synchronous way.
    """
    if isinstance(final_state, string_types):
        final_state = sm[final_state]

    event_queue = Queue.Queue()
    def dispatch_with_queue(event):
        event_queue.put(event)

    sm._dispatch = sm.dispatch
    sm.dispatch = dispatch_with_queue

    sm.initialize()
    def finished():
        return sm.leaf_state is final_state or sm.leaf_state.is_substate(final_state)

    while not finished():
        event = event_queue.get()
        sm._dispatch(event)

    print('finished')
    sm.dispatch = sm._dispatch
    del(sm._dispatch)
