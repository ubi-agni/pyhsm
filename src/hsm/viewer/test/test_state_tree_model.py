from __future__ import print_function

import pytest
from hsm.core import StateMachine, Container
from hsm.introspection import IntrospectionServer
from pyhsm_msgs.msg import HsmStructure, HsmState
from hsm.viewer.state_tree_model import StateTreeModel
from hsm.viewer.state_node import *


def to_list(tree, parent=None, indent=0):
    result = []
    for child in tree.children(parent):
        result.append("  " * indent + tree.get_value(child, StateTreeModel.LABEL))
        result += to_list(tree, child, indent + 1)
    return result


def test_dummy():
    dummy = DummyStateNode("path")
    with pytest.raises(AttributeError):
        dummy.initial


def test_update():
    msg = HsmState(path="A", initial="i1")
    state = RootStateNode(msg, "", "")
    assert not state.update(msg)
    msg = HsmState(path="A", initial="i2")
    assert state.update(msg)
    assert state.initial == "i2"


def test_dummy_splitting():
    tree = StateTreeModel()
    assert to_list(tree) == []

    tree._create_or_split_dummy("a/b/c")
    assert to_list(tree) == ["a/b/c"]

    tree._create_or_split_dummy("a/b/c/d")
    assert to_list(tree) == ["a/b/c", "  d"]

    tree._create_or_split_dummy("a/b")
    assert to_list(tree) == ["a/b", "  c", "    d"]


def test_find():
    tree = StateTreeModel()
    for path in ["a1", "a2", "a3"]:
        tree._create_or_split_dummy(path)
    tree._create_or_split_dummy("a2/b/c")
    tree._create_or_split_dummy("a2/b/c/d")

    for path in ["a1", "a2", "a3"]:
        item = tree.find_node(path)
        assert item is not None
        assert tree.label(item) == path

    a2 = tree.find_node("a2")
    for path, label in zip(["b/c", "b/c/d"], ["b/c", "d"]):
        item = tree.find_node(path, parent=a2)
        assert item is not None
        assert tree.label(item) == label


def test_find_next():
    m = StateMachine("machine")
    m.add_states("A", "B", "C")
    l = ["machine", "  A", "  B", "  C"]

    tree = StateTreeModel()
    msg = HsmStructure(states=IntrospectionServer._state_msgs(m))

    tree.update_tree(msg, "server1")
    assert to_list(tree) == l
    tree.update_tree(msg, "server1")
    assert to_list(tree) == l

    tree.update_tree(msg, "server2")
    assert to_list(tree) == l * 2
    tree.update_tree(msg, "server2")
    assert to_list(tree) == l * 2


def test_find_next_nested():
    m = StateMachine("M")
    # state A/B contains the separating slash!
    m.add_states(Container("A/B"), Container("A"), Container("B"))  # 3 different states
    m["A/B"].add_states("C")
    m["A"].add_states("B")
    m["B"].add_states("D")

    msg = HsmStructure(states=IntrospectionServer._state_msgs(m))
    tree = StateTreeModel()
    tree.update_tree(msg, "/server")
    print(to_list(tree))
    assert to_list(tree) == ["M", "  A/B", "    C", "  A", "    B", "  B", "    D"]

    ab = tree.find_node("M/A/B")
    b = tree.find_node("B", parent=tree.find_node("M/A"))
    # states A/B and A / B are different
    assert tree.state(ab) != tree.state(b)
    assert tree.label(ab) == "A/B"
    assert tree.label(b) == "B"
    # but the share the same path
    # TODO To (reliably) distinguish them by path, we need to send list of labels describing the path
    assert tree.path(ab) == tree.path(b)
