class HierarchicalDict(object):
    def __init__(self, parent=None, *args, **kwargs):
        self._dict = dict()
        self.parent = parent
        self.declare(*args, **kwargs)

    def declare(self, *args, **kwargs):
        self._dict.update(*args, **kwargs)

    # __getattr__ works for attributes that are not predefined methods
    def __getattr__(self, key):
        while self is not None:
            try:
                return self._dict[key]
            except KeyError:
                self = self.parent

    def __setattr__(self, key, value):
        if key in self.__dict__:
            raise KeyError("%s cannot be written" % key)
        elif key in self._dict:
            self._dict[key] = value
        elif self.parent is None:
            raise KeyError("%s was not declared" % key)
