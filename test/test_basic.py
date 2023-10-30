from __future__ import print_function

import hsm


class SM(hsm.StateMachine):
    def __init__(self):
        super(SM, self).__init__("SM")
        self.A = self.add_state("A", initial=True)
        self.B = self.add_state("B")

        self.A.add_handler("event", self.handler)
        self.A.add_transition
        self.initialize()

    def handler(self, event):
        print("Processing event '{}' in state '{}'".format(event.name, self.leaf_state.name))
        self._transition_to(self.B, event)


if __name__ == "__main__":
    sm = SM()
    sm.dispatch("event")
    sm.dispatch("event")
