class HierarchicalDict(object):
    """Hierarchical dictionary

    Variables can be get / set via attribute syntax: dict.var.
    Variables need to be declared before they can be set.
    If not defined in current dictionary, search proceeds in parent dictionary.
    """

    def __init__(self, parent=None, **kwargs):
        """Initialize a new hierarchical dictionary with the given parent
           and declare the keys given by remaining keyword arguments.
        """
        self.__dict__.update(_dict=dict(), parent=parent)
        self.declare(**kwargs)

    def declare(self, **kwargs):
        """Declare variables (and their initial values) using the given keyword arguments.

        Will not overwrite already declared variables.
        """
        # replace previous _dict with new one
        old_dict = self._dict
        self.__dict__.update(_dict=dict(**kwargs))
        # restore previously existing values
        self._dict.update(old_dict)

    # __getattr__ works for attributes that are not predefined methods
    def __getattr__(self, key):
        """Retrieve class attribute or variable value from dictionary with given key"""
        if key in self.__dict__:
            return self.key
        return self.__getitem__(key)

    def __setattr__(self, key, value):
        """Assign the given value to an (existing) class attribute or an (existing) dictionary variable"""
        if key in self.__dict__:
            self.__dict__[key]=value
        else:
            self.__setitem__(key, value)

    def __getitem__(self, key):
        """Retrieve a variable value from the dictionary.

        If not found in the current dictionary, search propagates the parent hierarchy.
        Finally, if key is not defined in the hierarchy, KeyError is raised.
        """
        while self is not None:
            try:
                return self._dict[key]
            except KeyError:
                self = self.parent
        raise KeyError('%s was not declared' % key)

    def __setitem__(self, key, value):
        """Assign the given value to the dictionary variable given by key.

        As for retrieval, search for key propagates upwards the parent hierarchy.
        """
        while self is not None:
            if key in self._dict:
                self._dict[key] = value
                return
            else:
                self = self.parent
        raise KeyError('%s was not declared' % key)