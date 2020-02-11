"""
Helper functions for creating the main window's GUI elements.
"""

import os

from .gtk_wrap import Gtk, GdkPixbuf


IMG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'img', '')
"""Directory for loading images relative to this file."""

LABEL_SPACER = '   '
"""Text to get invisible spacing using labels."""


def build_tree_model():
    """Return an empty tree model."""
    tree_model = Gtk.TreeStore(str, int)
    return tree_model


def build_list_model(data=None):
    """Return a list model optionally filled with the given data."""
    list_model = Gtk.ListStore(str)
    if data is not None:
        for row in data:
            list_model.append(row)
    return list_model


def build_label(text):
    """Return a read-only label with the given text."""
    label = Gtk.Label(text)
    label.set_selectable(False)
    return label


def build_button(label=None, icon=None, icon_path=None, tooltip=None):
    """
    Return a button with the given attributes.
    If both ``icon`` and ``icon_path`` are given, ``icon`` takes priority.
    """
    button = Gtk.Button()
    if icon is None and icon_path is not None:
        icon = build_image(icon_path)
    if icon is not None:
        button.set_image(icon)
    if label is not None:
        button.set_label(label)
    if tooltip is not None:
        button.set_tooltip_text(tooltip)
    return button


def build_toggle_button(label=None):
    """Return a toggle button with the given label."""
    toggle_button = Gtk.ToggleButton.new()
    if label is not None:
        toggle_button.set_label(label)
    return toggle_button


def build_combo_box(model=None):
    combo_box = Gtk.ComboBox.new()
    combo_box.set_model(model)
    return combo_box


def build_toolbar():
    """Return a horizontal text toolbar with an overflow arrow enabled."""
    toolbar = Gtk.Toolbar()
    toolbar.set_orientation(Gtk.Orientation.HORIZONTAL)
    # Show overflow menu
    toolbar.set_show_arrow(True)
    toolbar.set_style(Gtk.ToolbarStyle.BOTH)
    return toolbar


def build_tool_item(content=None, tooltip=None):
    """Return a tool item with the given content."""
    tool_item = Gtk.ToolItem.new()
    if content is not None:
        tool_item.add(content)
    if tooltip is not None:
        tool_item.set_tooltip_text(tooltip)
    return tool_item


def build_tool_label(label):
    """Return a label wrapped in a ``Gtk.ToolItem``."""
    label = build_label(label)
    return build_tool_item(label)


def build_image(image_path, width=16, height=16):
    """Return an image loaded from the given path and scaled to ``size``."""
    pixbuf = GdkPixbuf.Pixbuf.new_from_file(image_path)
    pixbuf = pixbuf.scale_simple(width, height, GdkPixbuf.InterpType.BILINEAR)
    image = Gtk.Image()
    image.set_from_pixbuf(pixbuf)
    return image


def build_tool_button_no_border(label=None, icon_path=None, tooltip=None):
    """Return a ``Gtk.ToolButton`` with the given attributes."""
    button = Gtk.ToolButton(label=label)
    if icon_path is not None:
        icon = build_image(icon_path, 20, 20)
        button.set_icon_widget(icon)
    if tooltip is not None:
        button.set_tooltip_text(tooltip)
    return button


def build_tool_button(*args, **kwargs):
    """
    Return a button wrapped in a ``Gtk.ToolItem`` with the given attributes
    (see ``build_button`` for details).
    """
    button = build_button(*args, **kwargs)
    return build_tool_item(button)


def build_menu_item(label, has_mnemonic=False):
    """
    Return a menu item with the given label. If ``has_mnemonic`` is ``True``, the mnemonic
    character is indicated by an underscore in front of it.
    """
    if has_mnemonic:
        return Gtk.MenuItem.new_with_mnemonic(label)
    else:
        return Gtk.MenuItem.new_with_label(label)


def build_integer_spin_button(value, lower, upper=1337, step_incr=1, page_incr=5, page_size=0):
    """Return a spin button with adjustments for small whole numbers."""
    adjustment = Gtk.Adjustment(value=value, lower=lower, upper=upper,
                                step_incr=step_incr, page_incr=page_incr, page_size=page_size)
    spin_button = Gtk.SpinButton()
    spin_button.set_adjustment(adjustment)

    # Only accept numeric input
    spin_button.set_numeric(True)
    return spin_button


def build_box(spacing=0, is_vertical=True):
    """Return a box with vertical or horizontal layout depending on ``is_vertical``."""
    box = Gtk.Box()
    box.set_spacing(spacing)
    if is_vertical:
        box.set_orientation(Gtk.Orientation.VERTICAL)
    else:
        box.set_orientation(Gtk.Orientation.HORIZONTAL)
    return box


def build_paned_frame(label=None):
    """Return a frame for use in a ``Gtk.Paned`` with the given label."""
    # Put `Paned` child in frame as recommended by
    # https://lazka.github.io/pgi-docs/Gtk-3.0/classes/Paned.html#Gtk.Paned
    # "Often, it is useful to put each child inside a Gtk.Frame with
    # the shadow type set to Gtk.ShadowType.IN so that the gutter
    # appears as a ridge." (retrieved 2019-03-08)
    #
    # We only do this for the detail view because the graph view
    # should have all the space it can get.
    frame = Gtk.Frame.new(label)
    frame.set_shadow_type(Gtk.ShadowType.IN)
    return frame


def build_statusbar():
    """Return a statusbar."""
    return Gtk.Statusbar()