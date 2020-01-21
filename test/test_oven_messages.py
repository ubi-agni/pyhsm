from __future__ import print_function

import sys

import rospy

import pyhsm_msgs.msg as msgs

# Use `rostopic echo /oven_msg` to listen for the message we send here.
# The topic may not be 'oven_msg' if an argument with another name is supplied.

# Oven HSM (initial states with asterisks):
#
# Oven -+- *Door closed* -+- *Off*
#       |                 +- Heating -+- *Baking*
#       |                             +- Toasting
#       +- Door open


DEFAULT_TOPIC_NAME = 'oven_msg'


class OvenMessage(object):

    @staticmethod
    def new():
        """Return our oven HSM as a message."""
        structure_msg = OvenMessage.create_structure_msg('/')
        return structure_msg

    @staticmethod
    def create_structure_msg(prefix):
        """Return the structure message of our oven HSM."""
        states = OvenMessage.create_state_msgs()
        return msgs.HsmStructure(prefix, states)

    @staticmethod
    def create_state_msgs():
        """Return a tuple of all states of our oven HSM."""
        # All tuples used here are ordered and need to stay ordered correctly
        # so the call to `zip` below works the way we want it to.
        paths = tuple('Oven' + path
                      for path in (
                              '',
                              '/Door closed',  # initial
                              '/Door closed/Off',  # initial
                              '/Door closed/Heating',
                              '/Door closed/Heating/Baking',  # initial
                              '/Door closed/Heating/Toasting',
                              '/Door open',
                      ))

        initial_states = (
            'Door closed',
            'Off',
            '',
            'Baking',
            '',
            '',
            '',
        )

        transitions = OvenMessage.create_transitions()

        assert len(paths) == len(initial_states) == len(transitions)

        states = tuple(
            msgs.HsmState(path, initial, state_transitions)
            for (path, initial, state_transitions) in zip(paths,
                                                          initial_states,
                                                          transitions))

        return states

    @staticmethod
    def create_transitions():
        """Create a tuple of transitions in order of the states."""
        # This tuple is ordered and needs to stay ordered correctly according
        # to the state paths in `create_state_msgs`.
        transitions = (
            # Oven
            (),

            # Oven/Door closed
            (
                msgs.HsmTransition(
                    ('toast',),
                    'Oven/Door closed/Heating/Toasting',
                    (),
                    (),
                    (),
                    (),
                ),
                msgs.HsmTransition(
                    ('bake',),
                    'Oven/Door closed/Heating/Baking',
                    (),
                    (),
                    (),
                    (),
                ),
                msgs.HsmTransition(
                    ('off', 'timeout'),
                    'Oven/Door closed/Off',
                    (),
                    (),
                    (),
                    (),
                ),
                msgs.HsmTransition(
                    ('open',),
                    'Oven/Door open',
                    (),
                    (),
                    (),
                    (),
                ),
            ),

            # Oven/Door closed/Off
            (),

            # Oven/Door closed/Heating
            (),

            # Oven/Door closed/Heating/Baking
            (),

            # Oven/Door closed/Heating/Toasting
            (),

            # Oven/Door open
            (
                msgs.HsmTransition(
                    ('close',),
                    # This was coded as a history state which by default maps
                    # to the state itself.
                    # In original code in ./test_oven.py:
                    #    # trigger transition to HISTORY state
                    #    door_open.add_transition('close', door_closed.HISTORY)
                    'Oven/Door closed',
                    (),
                    (),
                    (),
                    (),
                ),
            ),
        )
        return transitions


def main(topic_name=DEFAULT_TOPIC_NAME):
    rospy.init_node('oven')

    server = rospy.Publisher(
        name=topic_name,
        data_class=msgs.HsmStructure,
        queue_size=1,
        latch=True,
    )
    msg = OvenMessage.new()

    while True:
        # TODO First message fails to send
        server.publish(msg)

        input_ = raw_input("""Published structure msg to topic '{}'.
Press enter to publish again, send 'q' to quit...
""".format(topic_name))

        if input_ == 'q':
            break


if __name__ == '__main__':
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        topic_name = DEFAULT_TOPIC_NAME

    main(topic_name)
