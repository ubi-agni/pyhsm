Basic Concepts
==============

A state machine is composed from a set of states that are organized in an hierarchical fashion.

Incoming events are dispatched to the currently active state. If that state doesn't handle the event itself,
it is propagated to its parent state. If that one doesn't handle the event either, it propagates to the grand parent and so on.
If the root state didn't handle the event a warning is thrown.
