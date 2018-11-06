from hsm.core import State, Event, string_types
import rospy

import logging
_LOGGER = logging.getLogger("hsm.ros")


class _ROSEvent(object):
    def __init__(self, topic, msg_type, handler):
        self.topic = topic
        self.msg_type = msg_type
        self.handler = handler
        self.subscriber = None

def _subscribe_ros_events(state, event):
    # subscribe events
    for e in state._ros_events:
        _LOGGER.debug("subscribing to " + e.topic)
        e.subscriber = rospy.Subscriber(e.topic, e.msg_type, e.handler)


def _unsubscribe_ros_events(state, event):
    # unsubscribe events
    for e in state._ros_events:
        del e.subscriber
        _LOGGER.debug("unsubscribed from " + e.topic)


def _ros_subscribe(state, topic, msg_type, handler):
    # once, define _ros_events and register enter/exit handlers to actually subscribe/unsubscribe events
    if not hasattr(state, '_ros_events'):
        # define _ros_events attribute
        state._ros_events = []
        # on enter, subscribe to topics
        state.add_handler('enter', lambda event: _subscribe_ros_events(state, event))
        # on exit, unsubscribe
        state.add_handler('exit', lambda event: _unsubscribe_ros_events(state, event))

    # a string "handler" is interpreted as an event to be triggered
    if isinstance(handler, string_types):
        name = handler  # need to use another name!
        handler = lambda data: state.root.dispatch(Event(name, msg=data))

    # actually register the event
    state._ros_events.append(_ROSEvent(topic, msg_type, handler))


# augment State class with ros_subscribe method
setattr(State, "ros_subscribe", _ros_subscribe)
