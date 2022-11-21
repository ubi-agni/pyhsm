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
from six import iteritems, string_types
from six.moves.queue import Queue, Empty

import collections
from threading import Thread
import sys
import signal
import logging

# Configure logging defaults
# (rospy logger config disables all unconfigured loggers other than rosout)
_LOGGER = logging.getLogger("hsm")
_LOGGER.setLevel(logging.INFO)
handler = logging.StreamHandler(stream=sys.stdout)
handler.setFormatter(logging.Formatter(fmt='[%(levelname)s] %(message)s'))
_LOGGER.addHandler(handler)

_LOGGER = _LOGGER.getChild("core")

any_event = object()


class OrderedSet(collections.OrderedDict, collections.MutableSet):
    def update(self, *args):
        for s in args:
            for e in s:
                self.add(e)

    def add(self, elem):
        self[elem] = None

    def discard(self, elem):
        self.pop(elem, None)

    def __le__(self, other):
        return all(e in other for e in self)

    def __lt__(self, other):
        return self <= other and self != other

    def __ge__(self, other):
        return all(e in self for e in other)

    def __gt__(self, other):
        return self >= other and self != other

    def __repr__(self):
        return 'OrderedSet([%s])' % (', '.join(map(repr, self.keys())))

    def __str__(self):
        return '{%s}' % (', '.join(map(repr, self.keys())))

    difference = property(lambda self: self.__sub__)
    difference_update = property(lambda self: self.__isub__)
    intersection = property(lambda self: self.__and__)
    intersection_update = property(lambda self: self.__iand__)
    issubset = property(lambda self: self.__le__)
    issuperset = property(lambda self: self.__ge__)
    symmetric_difference = property(lambda self: self.__xor__)
    symmetric_difference_update = property(lambda self: self.__ixor__)
    union = property(lambda self: self.__or__)


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


def bind(instance, function):
    """ Turn a function to a bound method on an instance
    :param instance: some object
    :param function: unbound method, i.e. a function that takes `self` argument
                     that you now want to be bound to this class as a method
    """
    return function.__get__(instance, instance.__class__)


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

    def get(self, state, event):
        for key in [event.name, any_event]:
            for transition in self[key]:
                if transition['condition'](state, event) is True:
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
            def on_enter(self, event):
                print('Running state entered')

            def on_jump(self, event):
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
                self.add_handler(trigger[3:], getattr(self,trigger))

    def add_handler(self, events, func, prepend=False):
        """ Add a new event callback.

        :param trigger: name of triggering event
        :type trigger: str
        :param func: callback function
        :type func: callable
        """
        for event in listify(events):
            handlers = self._handlers.get(event, [])
            if prepend:
                handlers.insert(0, func)
            else:
                handlers.append(func)
            if len(handlers) == 1:  # initially create dict entry
                self._handlers[event] = handlers

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
        # when calling enter/exit handlers pass the original event
        is_enter_exit_event = event.name in ['exit', 'enter']
        e = event.userdata['source_event'] if is_enter_exit_event else event

        # HACK: State doesn't provide a leaf_state, while self.parent might be None
        current = self.leaf_state if self.parent is None else self.parent.leaf_state
        handler = self._handlers.get(event.name, None)
        transition = None if is_enter_exit_event else self._transitions.get(current, event)
        if handler is not None and transition is not None:
            raise Exception("Both, event handler and transition defined for event '{}' in state '{}'".format(event.name, self.name))

        if handler:
            if e is not None:
                e.propagate = False
            _call(handler, e)
        elif transition is not None:
            sm = self.root
            to_state = transition['to_state']
            if isinstance(to_state, _History):
                to_state = to_state.parent.history_state

            transition['before'](self, event)
            top_state = sm._exit_states(event, self, to_state)
            transition['action'](self, event)
            sm._enter_states(event, top_state, to_state)
            sm.call_transition_cbs(current, to_state)
            # Why is self.parent.leaf_state passed here?
            # transition['after'](self.parent.leaf_state, event)
            transition['after'](to_state, event)
            e.propagate = False

        # Never propagate exit/enter events, even if propagate is set to True
        if not is_enter_exit_event and self.parent and e.propagate:
            self.parent._on(event)

    def _nop(self, state, event):
        return True


class Container(State):
    """Container interface.

    Containers allow for hierarchical nesting of states.
    """

    def __init__(self, name):
        super(Container, self).__init__(name)
        self.states = OrderedSet()
        # current local state
        self.state = None
        # previous local state (for transitions to history)
        self._prev_state = None
        # special history state
        self.HISTORY = _History(self)

    def __getitem__(self, key):
        def find_by_name(name):
            for state in self.states:
                if state.name == name:
                    return state

        # TODO: Unify separator character with introspection, which uses '/'
        state = self
        keys = [None, key]
        while state is not None and len(keys) > 1:
            keys = keys[1].split('.', 1)
            state = find_by_name(keys[0])
        if state is None:
            raise IndexError('Unknown state {}'.format(key))
        return state

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

    def get_initial_states(self):
        initial = self.initial_state
        return [] if initial is None else [initial]

    def get_active_states(self):
        return [] if self.state is None else [self.state]

    def add_transition(self, events, target_state, *args, **kwargs):
        # handle string names: retrieve State instances
        if not isinstance(target_state, State):
            target_state = self[target_state]
        super(Container, self).add_transition(events, target_state, *args, **kwargs)

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
        state = self
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

    def initialize(self):
        """Initialize states in the state machine.

        After a state machine has been created and all states are added to it,
        :func:`initialize` has to be called.

        If using nested state machines (HSM),
        :func:`initialize` has to be called on a root
        state machine in the hierarchy.

        """
        states = collections.deque()
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
        self._enter_states(Event(None), None, self.state)
        self.call_transition_cbs(None, self.leaf_state)

    def register_transition_cb(self, transition_cb, *args):
        """Adds a transition callback to this container."""
        self._transition_cbs.append((transition_cb, args))

    def unregister_transition_cb(self, transition_cb, *args):
        """Adds a transition callback to this container."""
        self._transition_cbs.remove((transition_cb, args))

    def call_transition_cbs(self, from_state, to_state):
        """Calls the registered transition callbacks.
        Callback functions are called with two arguments in addition to any
        user-supplied arguments:
         - userdata
         - a list of active states
         """
        for (cb, args) in self._transition_cbs:
            cb(from_state, to_state, *args)

    def dispatch(self, event):
        """Dispatch an event to a state machine.

        If using nested state machines (HSM), it has to be called on a root
        state machine in the hierarchy.

        :param event: Event to be dispatched
        :type event: :class:`.Event`

        """
        if isinstance(event, string_types):
            event = Event(event)
        if event.name == '__TRANSITION__':
            self._transition_to(event.userdata['to_state'], event=None)
        else:
            event._machine = self
            self.leaf_state._on(event)

    def _transition_to(self, to_state, event):
        """manual transition to given state"""
        leaf_state_before = self.leaf_state
        top_state = self._exit_states(event, leaf_state_before, to_state)
        self._enter_states(event, top_state, to_state)
        self.call_transition_cbs(leaf_state_before, self.leaf_state)

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
            state._on(exit_event)
            state.parent._prev_state = state
            state.parent.state = None
            state = state.parent
        return state

    def _enter_states(self, event, top_state, to_state):
        if to_state is None:
            return

        # descend to initial state
        while isinstance(to_state, Container):
            initial = to_state.initial_state
            if initial is None:
                break
            to_state = initial

        # find path from top_state to to_state
        path = []
        state = to_state
        while state.parent and state != top_state:
            path.append(state)
            state = state.parent
        if top_state is None:
            path.append(self)

        for state in reversed(path):
            _LOGGER.debug('entering %s', state.name)
            enter_event = Event('enter', propagate=False, source_event=event)
            enter_event._machine = self
            state._on(enter_event)

            # remember current state in parent
            if state.parent is not None:
                state.parent.state = state


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
        machines = collections.deque()
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
            if not isinstance(result, Event):
                result = Event(result)
            self.state.root.dispatch(result)


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

    def _on_enter(self, event):
        self.preempt_requested = False
        self._thread = ProcessorThread(self, event)
        self._thread.start()

    def _on_exit(self, event):
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

    event_queue = Queue()
    def _dispatch_with_queue(event):
        event_queue.put(event)

    _dispatch = sm.dispatch
    sm.dispatch = _dispatch_with_queue

    sm.initialize()
    def _finished():
        return sm.leaf_state is final_state or sm.leaf_state.is_substate(final_state)

    signal_chain = dict()
    def _signal_handler(sig, frame):
        print(' Finishing on signal ' + str(sig))
        sm.dispatch(Event('__TRANSITION__', to_state=final_state))

    # install signal handler
    for s in [signal.SIGINT, signal.SIGTERM]:
        signal_chain[s] = signal.signal(s, _signal_handler)

    if sys.version_info[0] > 2:
        args = []  # blocking call for python 3
    else:
        args = [True, 100]  # non-blocking call for python 2, which ignores signals (Ctrl-C) otherwise

    while not _finished():
        try:
            event = event_queue.get(*args)
            _dispatch(event)
        except Empty:
            pass

    # restore signal handler
    for s in [signal.SIGINT, signal.SIGTERM]:
        signal.signal(s, signal_chain[s])

    # restore dispatch()
    sm.dispatch = _dispatch
