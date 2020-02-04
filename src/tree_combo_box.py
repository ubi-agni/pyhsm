"""
A combo box with a tree view.

Custom implementation as ``wx.lib.combotreebox.ComboTreeBox`` cannot be added to toolbars.
"""

import wx
from wx.combo import ComboCtrl, ComboPopup


class TreeComboBox(ComboCtrl):
    """A hierarchical combo box containing a ``wx.TreeCtrl`` combo popup."""

    def __init__(self, parent, id=wx.ID_ANY, size=wx.DefaultSize, style=0,
                 tree_style=wx.TR_HAS_BUTTONS):
        ComboCtrl.__init__(self, parent, id, size=size, style=style)
        self.__tree_popup = TreeCtrlComboPopup(style=tree_style)
        self.SetPopupControl(self.__tree_popup)
        self.__tree_ctrl = self.__tree_popup.GetControl()

    def Append(self, *args, **kwargs):
        # Even though we have a ``wx.ComboBox``-like interface, we only provide tree methods.
        raise NotImplementedError('Please call ``AddRoot`` or ``AppendItem`` to add items.')

    # API for the internal ``wx.TreeCtrl``
    # ------------------------------------

    def AddRoot(self, *args, **kwargs):
        """Public interface to the internal tree's ``wx.TreeCtrl.AddRoot``."""
        return self.__tree_ctrl.AddRoot(*args, **kwargs)

    def AppendItem(self, *args, **kwargs):
        """Public interface to the internal tree's ``wx.TreeCtrl.AppendItem``."""
        return self.__tree_ctrl.AppendItem(*args, **kwargs)

    def FindItemWithClientData(self, text):
        """Return the item in the tree with the given ``text`` as client data
        or the tree's root if none was found.
        """
        found = self.__tree_popup.FindItemWithClientData(self.GetRootItem(), text)
        if found:
            return found
        else:
            # This also returns a new root if there is none.
            return self.GetRootItem()

    def Expand(self, *args, **kwargs):
        """Public interface to the internal tree's ``wx.TreeCtrl.Expand``."""
        return self.__tree_ctrl.Expand(*args, **kwargs)

    def ExpandAll(self, *args, **kwargs):
        """Public interface to the internal tree's ``wx.TreeCtrl.ExpandAll``."""
        return self.__tree_ctrl.ExpandAll(*args, **kwargs)

    def GetRootItem(self, *args, **kwargs):
        """Public interface to the internal tree's ``wx.TreeCtrl.GetRootItem``."""
        return self.__tree_ctrl.GetRootItem(*args, **kwargs)


class TreeCtrlComboPopup(ComboPopup):
    """``wx.ComboPopup`` implemented with an internal ``wx.TreeCtrl``."""
    # Some functions adapted from
    # https://github.com/wxWidgets/wxPython-Classic/blob/19571e1ae65f1ac445f5491474121998c97a1bf0/demo/ComboCtrl.py
    # Retrieved 2020-02-04.

    def __init__(self, style):
        # Even though it is not recommended to overwrite this function, we need
        # to so we are able to pass the given style to our tree.
        ComboPopup.__init__(self)
        self.__style = style

    def Create(self, parent):
        self.__tree_ctrl = wx.TreeCtrl(parent, style=self.__style)
        self.__tree_ctrl.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        return True

    def GetControl(self):
        return self.__tree_ctrl

    def GetStringValue(self):
        item = self.__tree_ctrl.GetSelection()
        item_data = self.__tree_ctrl.GetItemData(item)
        return item_data.GetData()

    def SetStringValue(self, text):
        # This assumes that item client data strings are unique.
        # TODO Use an internal reverse dictionary instead; much simpler and more efficient!
        root = self.__tree_ctrl.GetRootItem()
        if not root:
            return

        found = self.FindItemWithClientData(root, text)
        if found:
            self.__tree_ctrl.SelectItem(found)

    def FindItemWithClientData(self, parent_item, text):
        """Iterate through the tree data starting from ``parent_item`` until an
        item's client data matches the given ``text`` and return that item.
        """
        item_data = self.__tree_ctrl.GetItemData(parent_item)
        if item_data.GetData() == text:
            return parent_item

        item_stack = [parent_item]
        while item_stack:
            parent_item = item_stack.pop()

            if self.__tree_ctrl.ItemHasChildren(parent_item):
                item, cookie = self.__tree_ctrl.GetFirstChild(parent_item)
                while item:
                    item_stack.append(item)
                    item_data = self.__tree_ctrl.GetItemData(item)
                    if item_data.GetData() == text:
                        return item
                    item, cookie = self.__tree_ctrl.GetNextChild(parent_item, cookie)

        return wx.TreeItemId()

    def OnLeftDown(self, evt):
        """Select an element in the tree and close the popup."""
        item, flags = self.__tree_ctrl.HitTest(evt.GetPosition())
        if item and flags & wx.TREE_HITTEST_ONITEMLABEL:
            self.__tree_ctrl.SelectItem(item)
            self.Dismiss()
        evt.Skip()
