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
        self._setattr('_dict', dict())
        self._setattr('parent', parent)
        self.declare(**kwargs)

    def _setattr(self, key, value):
        """Assign a new variable (not dictionary-/userdata!) with given name key and given value.

        Use this to assign new variables as __setattr__ is overwritten for dict-behaviour.
        If a variable with a key that coincides with a dictionary key is assigned using this method, the dictionary key
        is not accessible and a KeyError will be raised on the next call of declare().
        """
        super(HierarchicalDict, self).__setattr__(key, value)

    def declare(self, **kwargs):
        """Declare variables (and their initial values) using the given keyword arguments.

        Will not overwrite already declared variables.
        """
        old_dict = self._dict.copy()
        self._dict.update(**kwargs)
        for key in old_dict:
            self._dict[key] = old_dict[key]

    # __getattr__ works for attributes that are not predefined methods
    def __getattr__(self, key):
        """Retrieve either the class attribute or the value from the dictionary with the given key.

        If the given key corresponds to a class attribute, its value is returned.
        Otherwise, __getitem__() is called.
        """
        if key in self.__dict__:
            return self.key
        return self.__getitem__(key)

    def __setattr__(self, key, value):
        """Assign the given value to the class attribute or dictionary value given by key.

        If the given key corresponds to a class attribute, _setattr() is called and the behaviour is as usual.
        Otherwise, __setitem__() is called.
        """
        if key in self.__dict__:
            self._setattr(key, value)
        else:
            self.__setitem__(key, value)

    def __getitem__(self, key):
        """Retrieve either the value with the given key from the dictionary.

        When retrieving from a key that is not declared in the dictionary, the search propagates upwards through the
        parental hierarchy until either the key is found (and its value returned) or until no parent is left at which
        point a KeyError is raised.
        """
        while self is not None:
            if key in self._dict:
                return self._dict[key]
            else:
                self = self.parent
        raise KeyError('%s was not declared' % key)

    def __setitem__(self, key, value):
        """Assign the given value to the dictionary value given by key.

        When assigning to a key that is not declared in the dictionary, the search propagates upwards through the
        parental hierarchy until either the key is found (resulting in that parent's key's value being updated) or
        until no parent is left at which point a KeyError is raised.
        """
        while self is not None:
            if key in self._dict:
                self._dict[key] = value
                return
            else:
                self = self.parent
        raise KeyError('%s was not declared' % key)