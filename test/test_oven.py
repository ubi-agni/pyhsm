from __future__ import print_function

import threading
import time
import unittest

from hsm import State, Container, StateMachine, Event

class HeatingState(Container):
    def __init__(self, name):
        super(HeatingState, self).__init__(name)
        baking = State('Baking')
        self.add_state(baking, initial=True)
        toasting = State('Toasting')
        self.add_state(toasting)

    def on_enter(self, event):
        oven = event.userdata['oven']
        if not oven.timer.is_alive():
            oven.start_timer()
        print('Heating on')

    def on_exit(self, event):
        print('Heating off')


class Oven(object):
    TIMEOUT = 1

    def __init__(self):
        self.sm = self._get_state_machine()
        self.timer = threading.Timer(Oven.TIMEOUT, self.on_timeout)

    def _get_state_machine(self):
        oven = StateMachine('Oven')

        door_closed = Container('Door closed')
        oven.add_state(door_closed, initial=True)
        off = door_closed.add_state('Off', initial=True)
        heating = door_closed.add_state(HeatingState('Heating'))

        door_open = oven.add_state('Door open')

        door_closed.add_transition('toast', heating['Toasting'])
        door_closed.add_transition('bake', heating['Baking'])
        door_closed.add_transition(events=['off', 'timeout'], target_state=off)
        door_closed.add_transition('open', door_open)

        # trigger transition to HISTORY state
        door_open.add_transition('close', door_closed.HISTORY)

        # define enter/exit/event handlers as arbitrary callbacks
        door_open.add_handler('enter', self.on_open_enter)
        door_open.add_handler('exit', self.on_open_exit)

        oven.initialize()
        return oven

    def handler(self, msg):
        print(msg)

    @property
    def state(self):
        return self.sm.leaf_state.name

    def light_on(self):
        print('Light on')

    def light_off(self):
        print('Light off')

    def start_timer(self):
        self.timer.start()

    def bake(self):
        self.sm.dispatch(Event('bake', oven=self))

    def toast(self):
        self.sm.dispatch(Event('toast', oven=self))

    def open_door(self):
        self.sm.dispatch(Event('open', oven=self))

    def close_door(self):
        self.sm.dispatch(Event('close', oven=self))

    def on_timeout(self):
        print('Timeout...')
        self.sm.dispatch(Event('timeout', oven=self))
        self.timer = threading.Timer(Oven.TIMEOUT, self.on_timeout)

    def on_open_enter(self, event):
        print('Opening door')
        self.light_on()

    def on_open_exit(self, event):
        print('Closing door')
        self.light_off()


class OvenTest(unittest.TestCase):
    def test(self):
        oven = Oven()

        self.assertEquals(oven.state, 'Off')

        oven.bake()
        self.assertEquals(oven.state, 'Baking')

        oven.open_door()
        self.assertEquals(oven.state, 'Door open')

        oven.close_door()
        self.assertEquals(oven.state, 'Baking')

        time.sleep(Oven.TIMEOUT + 0.5)
        self.assertEquals(oven.state, 'Off')


if __name__ == '__main__':
    unittest.main()
