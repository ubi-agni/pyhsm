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
import logging
_LOGGER = logging.getLogger(__name__)
_LOGGER.addHandler(logging.NullHandler())


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


class TransitionsContainer(object):
    def __init__(self, machine):
        self._machine = machine
        self._transitions = collections.defaultdict(list)

    def add(self, key, transition):
        self._transitions[key].append(transition)

    def get(self, event):
        key = (self._machine.state, event.name)
        return self._get_transition_matching_condition(key, event)

    def _get_transition_matching_condition(self, key, event):
        from_state = self._machine.leaf_state
        for transition in self._transitions[key]:
            if transition['condition'](from_state, event) is True:
                return transition
        key = (self._machine.state, any_event)
        for transition in self._transitions[key]:
            if transition['condition'](from_state, event) is True:
                return transition
        return None


class Stack(object):
    def __init__(self, maxlen=None):
        self.deque = deque(maxlen=maxlen)

    def pop(self):
        return self.deque.pop()

    def push(self, value):
        self.deque.append(value)

    def peek(self):
        return self.deque[-1]

    def __repr__(self):
        return str(list(self.deque))


class State(object):
    """Represents a state in a state machine.

    `enter` and `exit` handlers are called whenever a state is entered or
    exited respectively. These action names are reserved only for this purpose.

    It is encouraged to extend this class to encapsulate a state behavior,
    similarly to the State Pattern.

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
        self._transitions = TransitionsContainer(self)
        self.state_stack = Stack(maxlen=StateMachine.STACK_SIZE)
        self.leaf_state_stack = Stack(maxlen=StateMachine.STACK_SIZE)

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
        '''Get the initial state in a state machine.

        :returns: Initial state in a state machine
        :rtype: |State|

        '''
        for state in self.states:
            if state.initial:
                return state
        return None

    def set_initial_state(self, state):
        '''Set an initial state in a state machine.

        :param state: Set this state as initial in a state machine
        :type state: |State|

        '''
        Validator(self).validate_set_initial(state)
        state.initial = True

    def get_active_states(self):
        """Get the subset of active states.

        @rtype: list of |State|
        """
        raise NotImplementedError()

    def add_transition(
            self, from_state, to_state, events, action=None,
            condition=None, before=None, after=None):
        '''Add a transition to a state machine.

        All callbacks take two arguments - `state` and `event`. See parameters
        description for details.

        It is possible to create conditional if/elif/else-like logic for
        transitions. To do so, add many same transition rules with different
        condition callbacks. First met condition will trigger a transition, if
        no condition is met, no transition is performed.

        :param from_state: Source state
        :type from_state: |State|
        :param to_state: Target state. If `None`, then it's an `internal
            transition <https://en.wikipedia.org/wiki/UML_state_machine
            #Internal_transitions>`_
        :type to_state: |State|, `None`
        :param events: List of events that trigger the transition
        :type events: |Iterable| of |Hashable|
        :param input: List of inputs that trigger the transition. A transition
            event may be associated with a specific input. i.e.: An event may
            be ``parse`` and an input associated with it may be ``$``. May be
            `None` (default), then every matched event name triggers a
            transition.
        :type input: `None`, |Iterable| of |Hashable|
        :param action: Action callback that is called during the transition
            after all states have been left but before the new one is entered.

            `action` callback takes two arguments:

                - state: Leaf state before transition
                - event: Event that triggered the transition

        :type action: |Callable|
        :param condition: Condition callback - if returns `True` transition may
            be initiated.

            `condition` callback takes two arguments:

                - state: Leaf state before transition
                - event: Event that triggered the transition

        :type condition: |Callable|
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

        '''
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
        # handle string names: retrieve State instances
        from_state = self[from_state]
        to_state = self[to_state]

        Validator(self).validate_add_transition(from_state, to_state, events, input)

        for event in events:
            key = (from_state, event)
            transition = {
                'from_state': from_state,
                'to_state': to_state,
                'action': action,
                'condition': condition,
                'before': before,
                'after': after,
            }
            self._transitions.add(key, transition)

    @property
    def leaf_state(self):
        '''Get the current leaf state.

        The :attr:`~.StateMachine.state` property gives the current,
        local state in a state machine. The `leaf_state` goes to the bottom in
        a hierarchy of states. In most cases, this is the property that should
        be used to get the current state in a state machine, even in a flat
        FSM, to keep the consistency in the code and to avoid confusion.

        :returns: Leaf state in a hierarchical state machine
        :rtype: |State|

        '''
        return self._get_leaf_state(self)

    def _get_leaf_state(self, state):
        while hasattr(state, 'state') and state.state is not None:
            state = state.state
        return state


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

    If there's no handler in states or transition for an event, it is silently
    ignored.

    If using nested state machines, all events should be sent to the root state
    machine.

    **Attributes:**

        .. attribute:: state

            Current, local state (instance of |State|) in a state machine.

        .. attribute:: stack

            Stack that can be used if the `Pushdown Automaton (PDA)
            <https://en.wikipedia.org/wiki/Pushdown_automaton>`_ functionality
            is needed.

        .. attribute:: state_stack

            Stack of previous local states in a state machine. With every
            transition, a previous state (instance of |State|) is pushed to the
            `state_stack`. Only :attr:`.StateMachine.STACK_SIZE` (32
            by default) are stored and old values are removed from the stack.

        .. attribute:: leaf_state_stack

            Stack of previous leaf states in a state machine. With every
            transition, a previous leaf state (instance of |State|) is pushed
            to the `leaf_state_stack`. Only
            :attr:`.StateMachine.STACK_SIZE` (32 by default) are
            stored and old values are removed from the stack.

        **leaf_state**
            See the :attr:`~.StateMachine.leaf_state` property.

        **root**
            See the :attr:`~.StateMachine.root` property.

    :param name: Human readable state machine name
    :type name: str

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

    STACK_SIZE = 32

    def __init__(self, name):
        super(StateMachine, self).__init__(name)
        self.state = None
        self._transition_cbs = []

    def _get_transition(self, event):
        machine = self.leaf_state.parent
        while machine:
            transition = machine._transitions.get(event)
            if transition:
                return transition
            machine = machine.parent
        return None

    def initialize(self):
        '''Initialize states in the state machine.

        After a state machine has been created and all states are added to it,
        :func:`initialize` has to be called.

        If using nested state machines (HSM),
        :func:`initialize` has to be called on a root
        state machine in the hierarchy.

        '''
        states = deque()
        states.append(self)
        validator = Validator(self)
        while states:
            machine = states.popleft()
            validator.validate_initial_state(machine)
            machine.state = machine.initial_state
            for child_state in machine.states:
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
        '''Dispatch an event to a state machine.

        If using nested state machines (HSM), it has to be called on a root
        state machine in the hierarchy.

        :param event: Event to be dispatched
        :type event: :class:`.Event`

        '''
        event._machine = self
        leaf_state_before = self.leaf_state
        leaf_state_before._on(event)
        transition = self._get_transition(event)
        if transition is None:
            return
        to_state = transition['to_state']
        from_state = transition['from_state']

        transition['before'](leaf_state_before, event)
        top_state = self._exit_states(event, from_state, to_state)
        transition['action'](leaf_state_before, event)
        self._enter_states(event, top_state, to_state)
        transition['after'](self.leaf_state, event)
        self.call_transition_cbs()

    def _exit_states(self, event, from_state, to_state):
        if to_state is None:
            return None
        state = self.leaf_state
        self.leaf_state_stack.push(state)
        while (state.parent and
                not (from_state.is_substate(state) and
                     to_state.is_substate(state)) or
                (state == from_state == to_state)):
            _LOGGER.debug('exiting %s', state.name)
            exit_event = Event('exit', propagate=False, source_event=event)
            exit_event._machine = self
            state._on(exit_event)
            state.parent.state_stack.push(state)
            state.parent.state = state.parent.initial_state
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
            state._on(enter_event)
            if state.parent is not None:
                state.parent.state = state


    def set_previous_leaf_state(self, event=None):
        '''Transition to a previous leaf state. This makes a dynamic transition
        to a historical state. The current `leaf_state` is saved on the stack
        of historical leaf states when calling this method.

        :param event: (Optional) event that is passed to states involved in the
            transition
        :type event: :class:`.Event`

        '''
        if event is not None:
            event._machine = self
        from_state = self.leaf_state
        try:
            to_state = self.leaf_state_stack.peek()
        except IndexError:
            return
        top_state = self._exit_states(event, from_state, to_state)
        self._enter_states(event, top_state, to_state)

    def revert_to_previous_leaf_state(self, event=None):
        '''Similar to :func:`set_previous_leaf_state`
        but the current leaf_state is not saved on the stack of states. It
        allows to perform transitions further in the history of states.

        '''
        self.set_previous_leaf_state(event)
        try:
            self.leaf_state_stack.pop()
        except IndexError:
            return

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

    def validate_add_transition(self, from_state, to_state, events, input):
        self._validate_from_state(from_state)
        self._validate_to_state(to_state)
        self._validate_events(events)

    def _validate_from_state(self, from_state):
        if from_state not in self._machine.states:
            msg = 'Unable to add transition from unknown state "{0}"'.format(
                from_state.name)
            self._raise(msg)

    def _validate_to_state(self, to_state):
        root_machine = self._machine.root
        if to_state is None:
            return
        elif to_state is root_machine:
            return
        elif not to_state.is_substate(root_machine):
            msg = 'Unable to add transition to unknown state "{0}"'.format(
                to_state.name)
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
