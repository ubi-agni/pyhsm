from __future__ import print_function
from hsm.core import StateMachine
from hsm.introspection import IntrospectionServer
from pyhsm_msgs.msg import HsmStructure
from hsm.viewer.state_tree_model import StateTreeModel


def to_list(tree, parent=None, indent=0):
    result = []
    for child in tree.children(parent):
        result.append('  ' * indent + tree.get_value(child, StateTreeModel.LABEL))
        result += to_list(tree, child, indent + 1)
    return result


def test_dummy_splitting():
    tree = StateTreeModel()
    assert to_list(tree) == []

    tree._create_or_split_dummy('a/b/c')
    assert to_list(tree) == ['a/b/c']

    tree._create_or_split_dummy('a/b/c/d')
    assert to_list(tree) == ['a/b/c', '  d']

    tree._create_or_split_dummy('a/b')
    assert to_list(tree) == ['a/b', '  c', '    d']


def test_find():
    tree = StateTreeModel()
    for path in ['a1', 'a2', 'a3']:
        tree._create_or_split_dummy(path)
    tree._create_or_split_dummy('a2/b/c')
    tree._create_or_split_dummy('a2/b/c/d')

    for path in ['a1', 'a2', 'a3']:
        item = tree.find_node(path)
        assert item is not None
        assert tree.label(item) == path

    a2 = tree.find_node('a2')
    for path, label in zip(['b/c','b/c/d'], ['b/c','d']):
        item = tree.find_node(path, parent=a2)
        assert item is not None
        assert tree.label(item) == label


def mockup_machine():
    m = StateMachine('machine')
    m.add_states('A', 'B', 'C')
    l = ['machine', '  A', '  B', '  C']
    return m, l


def test_find_next():
    tree = StateTreeModel()
    m, l = mockup_machine()
    msg = HsmStructure(states = IntrospectionServer._state_msgs(m))

    tree.update_tree(msg, 'server1')
    assert to_list(tree) == l
    tree.update_tree(msg, 'server1')
    assert to_list(tree) == l

    tree.update_tree(msg, 'server2')
    assert to_list(tree) == l * 2
    tree.update_tree(msg, 'server2')
    assert to_list(tree) == l*2
