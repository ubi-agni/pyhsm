from __future__ import print_function

import vampire.vam as xcf
import hsm, hsm.xcf
from RobotAPI import *


def example_cmds():
    cmds = RobotCmds().select("LeftArm")
    cmds.moveMode(mode="stp", space="cartesian")
    cmds.moveTo([50, 40, 30])
    cmds.moveMode(mode="stp", space="joint")
    cmds.posture("home")
    cmds.notify(id="done")
    return cmds


class MoveState(hsm.State):
    def __init__(self, name, mi, **kwargs):
        hsm.State.__init__(self, name, **kwargs)
        self.mi = mi

    def on_enter(self, event):
        print("sending move commands")
        self.mi.send(example_cmds().toString())


if __name__ == "__main__":
    mi_sim = xcf.createMemoryInterface("xcf:sim")
    mi_real = xcf.createMemoryInterface("xcf:ShortTerm")

    # create a state machine with two (nearly) identical states triggering a move command
    m = hsm.StateMachine("xcf_test")
    sim = m.add_state(MoveState("sim", mi_sim), initial=True)
    real = m.add_state(MoveState("real", mi_real))
    done = m.add_state("done")

    # if simulation command succeeded, transition from sim to real
    sim.xcf_subscribe(
        mi_sim,
        xpath="/EVENT[@sender='ArmServer' and @name='finished' and @id='done']",
        handler="finished",
    )
    sim.add_transition("finished", real)

    # if real command succeeded, transition to done state
    real.xcf_subscribe(
        mi_real,
        xpath="/EVENT[@sender='ArmServer' and @name='finished' and @id='done']",
        handler="finished",
    )
    real.add_transition("finished", done)

    # subscribe to ArmServer:errors on top-level of state machine for simplicity
    m.xcf_subscribe(
        mi_sim,
        xpath="/EVENT[@sender='ArmServer' and @name='error' and not(@interrupted)]",
        handler="ArmServer:error",
    )
    m.xcf_subscribe(
        mi_real,
        xpath="/EVENT[@sender='ArmServer' and @name='error' and not(@interrupted)]",
        handler="ArmServer:error",
    )
    m.add_transition("ArmServer:error", done, action=lambda state, event: print(event))

    hsm.run(m, final_state=done)
