from gtk_wrap import Gtk
import gui_element_builder as geb


class HorizontalToolbar(Gtk.Toolbar):
    """
    A horizontal toolbar with an overflow menu.

    Convenience wrapper around the original toolbar.
    """

    def __init__(self, *args, **kwargs):
        Gtk.Toolbar.__init__(self, *args, **kwargs)
        self.set_orientation(Gtk.Orientation.HORIZONTAL)
        # Show overflow menu
        self.set_show_arrow(True)
        # Show both text and buttons
        self.set_style(Gtk.ToolbarStyle.BOTH)

    def add(self, element):
        """Add an element and wrap it inside a ``Gtk.ToolItem`` if necessary."""
        if not isinstance(element, Gtk.ToolItem):
            element = geb.build_tool_item(element)
        Gtk.Toolbar.add(self, element)

    def add_spacer(self):
        """Add a small, invisible spacing element."""
        self.add(geb.build_tool_label(geb.LABEL_SPACER))
