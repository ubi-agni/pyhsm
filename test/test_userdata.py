from hsm.userdata import HierarchicalDict
import unittest


class TestHierarchicalDict(unittest.TestCase):
    def test_assign_and_retrieve(self):
        d = HierarchicalDict(None, a=None, b=1)
        self.assertEquals(d.b, 1)
        for value in [10, 3.14, None]:
            d.a = value
            self.assertEquals(d.a, value)
        d.b = False
        self.assertEquals(d.b, False)

    def test_undeclared_assign(self):
        d = HierarchicalDict()
        with self.assertRaises(KeyError):
            d.a = None

    def test_undeclared_retrieve(self):
        d = HierarchicalDict()
        with self.assertRaises(KeyError):
            d.a

    def test_assign_and_retrieve_with_parent(self):
        parent = HierarchicalDict(a=[], b="b")
        child = HierarchicalDict(parent, b=1)
        self.assertEquals(parent.b, "b")
        self.assertEquals(child.b, 1)
        self.assertEquals(child.a, [])

        child.a = 3  # overwrite parent's value
        self.assertEquals(parent.a, 3)
        self.assertEquals(child.a, 3)

        parent.b = 5  # do not overwrite child's value
        self.assertEquals(parent.b, 5)
        self.assertEquals(child.b, 1)


if __name__ == "__main__":
    unittest.main()
