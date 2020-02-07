from gtk_wrap import Gtk
import gui_element_builder as geb
from horizontal_toolbar import HorizontalToolbar
from .xdot import xdot


class GraphView(Gtk.Box):
    """Graph view containing a toolbar and interactive dot graph widget."""

    def __init__(self, model, *args, **kwargs):
        """Build the graph view and its GUI elements."""
        Gtk.Box.__init__(self, *args, **kwargs)
        self.set_spacing(4)
        self.set_orientation(Gtk.Orientation.VERTICAL)

        # Toolbar
        self.toolbar = GraphViewToolbar(model)
        self.pack_start(self.toolbar, expand=False, fill=False, padding=0)

        # Dot Widget
        self.dot_widget = xdot.DotWidget()
        self.pack_start(self.dot_widget, expand=True, fill=True, padding=0)


class GraphViewToolbar(HorizontalToolbar):
    """A toolbar for actions related to the graph view."""

    HELP_BUTTON_PROXY_ID = 'help'
    SAVE_BUTTON_PROXY_ID = 'save'

    def __init__(self, model, *args, **kwargs):
        """Initialize the toolbar, its GUI elements and the callbacks."""
        HorizontalToolbar.__init__(self, *args, **kwargs)
        self.__setup_gui_elements(model)
        self.__connect_callbacks()

    def __setup_gui_elements(self, model):
        """Create all GUI elements and add them to the toolbar."""
        # Path filter combo box
        self.add(geb.build_label('Filter: '))
        self.path_filter_combo = geb.build_combo_box(model)
        self.add(self.path_filter_combo)

        # Depth spin button
        self.add(geb.build_label(geb.LABEL_SPACER + 'Depth: '))
        self.depth_spin_button = geb.build_integer_spin_button(-1, -1)
        self.add(self.depth_spin_button)

        # Toggle auto focus
        self.add_spacer()
        self.auto_focus_toggle = geb.build_toggle_button('Auto Focus')
        self.add(self.auto_focus_toggle)

        # Show implicit/transitions
        self.add_spacer()
        self.show_all_toggle = geb.build_toggle_button('Show Implicit')
        self.add(self.show_all_toggle)

        # Label width spin button
        self.add(geb.build_label(geb.LABEL_SPACER + 'Label Width: '))
        self.label_width_spin_button = geb.build_integer_spin_button(40, 1)
        self.add(self.label_width_spin_button)

        # TODO Small bug: If window is resized, overflow is sometimes not correctly checked
        #      and the arrow overlaps with a supposedly hidden item.
        #      Maybe emit the check-for-overflow-signal ourselves when the window is resized?

        # Help
        self.add_spacer()
        self.help_button = geb.build_button(label='Help')
        self.help_button_proxy = geb.build_menu_item('Help')
        help_tool_button = geb.build_tool_item(self.help_button)
        help_tool_button.set_proxy_menu_item(self.HELP_BUTTON_PROXY_ID, self.help_button_proxy)
        self.add(help_tool_button)

        # Save
        self.save_button = geb.build_button(label='Save')
        self.save_button_proxy = geb.build_menu_item('Save')
        save_tool_button = geb.build_tool_item(self.save_button)
        save_tool_button.set_proxy_menu_item(self.SAVE_BUTTON_PROXY_ID, self.save_button_proxy)
        self.add(save_tool_button)

    def __connect_callbacks(self):
        """Connect all callbacks to the GUI elements."""
        # FIXME
        pass
