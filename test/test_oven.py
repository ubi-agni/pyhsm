from __future__ import print_function

import threading, logging, time, rospy

from hsm import State, Container, StateMachine, Event
from hsm.introspection import IntrospectionServer

class HeatingState(Container):
    def __init__(self, name):
        super(HeatingState, self).__init__(name)
        baking = State('Baking')
        self.add_state(baking, initial=True)
        toasting = State('Toasting')
        self.add_state(toasting)

    def on_enter(self, event):
        oven = self.root
        if not oven.timer.is_alive():
            oven.start_timer()
        print('Heating on')

    def on_exit(self, event):
        print('Heating off')


class Oven(StateMachine):
    TIMEOUT = 5

    def __init__(self):
        self._init_machine()
        self.timer = threading.Timer(Oven.TIMEOUT, self.on_timeout)

    def _init_machine(self):
        super(Oven, self).__init__('Oven')

        door_closed = Container('Door closed')
        self.add_state(door_closed, initial=True)
        off = door_closed.add_state('Off', initial=True)
        heating = door_closed.add_state(HeatingState('Heating'))

        door_open = self.add_state('Door open')

        door_closed.add_transition('toast', heating['Toasting'])
        door_closed.add_transition('bake', heating['Baking'])
        door_closed.add_transition(events=['off', 'timeout'], target_state=off)
        door_closed.add_transition('open', door_open)

        # trigger transition to HISTORY state
        door_open.add_transition('close', door_closed.HISTORY)

        # define enter/exit/event handlers as arbitrary callbacks
        door_open.add_handler('enter', self.on_open_enter)
        door_open.add_handler('exit', self.on_open_exit)

        self.initialize()

    def handler(self, msg):
        print(msg)

    def light_on(self):
        print('Light on')

    def light_off(self):
        print('Light off')

    def start_timer(self):
        self.timer.start()

    def bake(self):
        self.dispatch(Event('bake', oven=self))

    def toast(self):
        self.dispatch(Event('toast', oven=self))

    def open_door(self):
        self.dispatch(Event('open', oven=self))

    def close_door(self):
        self.dispatch(Event('close', oven=self))

    def on_timeout(self):
        print('Timeout...')
        self.dispatch(Event('timeout', oven=self))
        self.timer = threading.Timer(Oven.TIMEOUT, self.on_timeout)

    def on_open_enter(self, event):
        print('Opening door')
        self.light_on()

    def on_open_exit(self, event):
        print('Closing door')
        self.light_off()


if __name__ == '__main__':
    rospy.init_node('oven')

    # enable logging
    logger = logging.getLogger("hsm")
    logger.setLevel(logging.DEBUG)
    logger.addHandler(logging.StreamHandler())

    oven = Oven()
    sis = IntrospectionServer('hsm_introspection', oven, '')
    sis.start()
    raw_input("Press a key to quit")
    sis.stop()
