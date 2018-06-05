from hsm.userdata import HierarchicalDict
import unittest

class TestHierarchicalDict(unittest.TestCase):
    def test_assign(self):
        d = HierarchicalDict(a=None)
        for value in [10, 3.14, None]:
            d.a = value
            self.assertEquals(d.a, value)

    def test_undeclared_assign(self):
        d = HierarchicalDict()
        with self.assertRaises(KeyError):
            d.a = None
