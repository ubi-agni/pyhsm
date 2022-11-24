from __future__ import print_function

from hsm import StateMachine, CallbackState
import time
import unittest


class Compute(StateMachine):
    TIMEOUT = 1

    def __init__(self):
        super(Compute, self).__init__("root")
        self.done_count = 0

        compute = self.add_state(CallbackState("compute", self.compute))
        idle = self.add_state("idle", initial=True)
        done = self.add_state("done")
        done.add_handler("enter", self.count)

        # switch between states
        compute.add_transition("switch", idle)
        compute.add_transition("done", done)
        idle.add_transition("switch", compute)

        self.register_transition_cb(self.on_transition)
        self.initialize()

    def compute(self, event):
        for i in range(5):
            time.sleep(0.1)
            print(i)
        print("ready")
        return "done"

    def count(self, event):
        self.done_count += 1

    def on_transition(self, from_state, to_state):
        print(to_state.name)

    @property
    def current(self):
        return self.leaf_state.name


class CallbackStateTest(unittest.TestCase):
    def test(self):
        sm = Compute()
        self.assertEquals(sm.current, "idle")

        sm.dispatch("switch")
        self.assertEquals(sm.current, "compute")
        time.sleep(0.3)

        sm.dispatch("switch")
        self.assertEquals(sm.current, "idle")
        self.assertEquals(sm.done_count, 0)

        sm.dispatch("switch")
        self.assertEquals(sm.current, "compute")

        time.sleep(1.0)
        self.assertEquals(sm.current, "done")
        self.assertEquals(sm.done_count, 1)


if __name__ == "__main__":
    unittest.main()
