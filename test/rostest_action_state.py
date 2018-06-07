from __future__ import print_function

from hsm import StateMachine, any_event
from hsm.action_state import ActionState
import grasping_msgs.msg, geometry_msgs.msg, shape_msgs.msg
import time
import rospy
import unittest
import logging


class GraspProvider(ActionState):
    def __init__(self):
        super(GraspProvider, self).__init__("grasp_provider", grasping_msgs.msg.GenerateGraspsAction)

    def _on_enter(self, state, event):
        """fill and trigger a goal request"""
        self.goal.config_name = "shadow_left_handed_limited"
        o = self.goal.object
        o.header.frame_id = "world"
        p = shape_msgs.msg.SolidPrimitive()
        p.type = p.CYLINDER
        p.dimensions = [0.25, 0.03]
        o.primitives.append(p)
        o.primitive_poses.append(geometry_msgs.msg.Pose())

        # call parent's method to actually trigger the request
        super(GraspProvider, self)._on_enter(state, event)

    def feedback_cb(self, feedback):
        pass

class Compute(StateMachine):
    TIMEOUT = 1

    def __init__(self):
        super(Compute, self).__init__("root")
        self.done_count = 0
        self.grasps = 0

        compute = self.add_state(GraspProvider(), initial=True)
        done = self.add_state("done")
        failed = self.add_state("failed")

        # switch between states
        self.add_transition(compute, done, "SUCCEEDED", after=self.show_result)
        self.add_transition(compute, done, any_event)
        self.add_transition(done, compute, "compute")

        self.register_transition_cb(self.on_transition)
        self.initialize()

    def show_result(self, state, event):
        self.grasps = len(event.userdata['result'].grasps)
        self.done_count += 1
        log.debug("{} grasps found".format(len(event.userdata['result'].grasps)))

    def on_transition(self):
        log.debug("entered state: {}".format(self.leaf_state.name))


class ActionStateTest(unittest.TestCase):
    def sequential_test_normal(self):
        sm = Compute()
        self.assertEquals(sm.leaf_state.name, "grasp_provider")

        # wait some time for a result
        time.sleep(3)
        self.assertEquals(sm.leaf_state.name, "done")
        self.assertTrue(sm.grasps > 0)
        self.assertEquals(sm.done_count, 1)

    def sequential_test_abort(self):
        sm = Compute()
        self.assertEquals(sm.leaf_state.name, "grasp_provider")

        # abort compute by triggering any event other than SUCCEEDED
        time.sleep(0.5)
        sm.dispatch("foo")
        self.assertEquals(sm.leaf_state.name, "done")

        # wait some more time for a result
        time.sleep(3)
        self.assertEquals(sm.grasps, 0)
        self.assertEquals(sm.done_count, 0)

    def sequential_test_abort_reenter(self):
        sm = Compute()
        self.assertEquals(sm.leaf_state.name, "grasp_provider")

        # abort compute by triggering any event other than SUCCEEDED
        time.sleep(0.5)
        sm.dispatch("foo")
        self.assertEquals(sm.leaf_state.name, "done")
        self.assertEquals(sm.grasps, 0)
        self.assertEquals(sm.done_count, 0)

        # immediately re-enter compute
        sm.dispatch("compute")
        self.assertEquals(sm.leaf_state.name, "grasp_provider")

        # wait some more time for a result
        time.sleep(3)
        self.assertTrue(sm.grasps > 0)
        self.assertEquals(sm.done_count, 1)

    def sequential_tests(self):
        for name in sorted(dir(self)):
            if name.startswith("sequential_test"):
                yield name, getattr(self, name)

    def test_sequentially(self):
        for name, func in self.sequential_tests():
            print("running", name)
            func()


if __name__ == '__main__':
    # Not sure, why we need to manually setup logging
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter('[%(name)s] %(asctime)s: %(message)s'))
    log = logging.getLogger("hsm.action_state")
    log.addHandler(handler)
    log.setLevel(logging.DEBUG)

    rospy.init_node("action_client")
    unittest.main()
