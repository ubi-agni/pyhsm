from hsm.core import State, Event, string_types
import vampire.vam as xcf

import logging
_LOGGER = logging.getLogger("hsm.xcf")


class _XCFEvent(object):
    def __init__(self, memory, xpath, actions, handler):
        self.memory = memory
        self.xpath = xpath
        self.actions = actions
        self.handler = handler
        self.subscription = None

def _subscribe_xcf_events(state, event):
    # subscribe events
    for e in state._xcf_events:
        _LOGGER.debug("subscribing to " + e.xpath)
        e.subscription = e.memory.subscribe(e.actions, e.xpath, e.handler)


def _unsubscribe_xcf_events(state, event):
    # unsubscribe events
    for e in state._xcf_events:
        e.memory.unsubscribe(e.subscription)
        e.memory.subscription = None
        _LOGGER.debug("unsubscribed " + e.xpath)


def _xcf_subscribe(state, memory, xpath, handler, actions=xcf.INSERT | xcf.REPLACE):
    # once, define _xcf_events and register enter/exit handlers to actually subscribe/unsubscribe events
    if not hasattr(state, '_xcf_events'):
        # define _xcf_events attribute
        state._xcf_events = []
        # on enter, subscribe to topics
        state.add_handler('enter', lambda event: _subscribe_xcf_events(state, event))
        # on exit, unsubscribe
        state.add_handler('exit', lambda event: _unsubscribe_xcf_events(state, event))

    # a string "handler" is interpreted as an event to be triggered
    if isinstance(handler, string_types):
        name = handler  # need to use another name!
        handler = lambda data: state.root.dispatch(Event(name, msg=data))

    # actually register the event
    state._xcf_events.append(_XCFEvent(memory, xpath, actions, handler))


# augment State class with ros_subscribe method
setattr(State, "xcf_subscribe", _xcf_subscribe)