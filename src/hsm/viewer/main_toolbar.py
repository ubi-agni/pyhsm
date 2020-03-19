import gui_element_builder as geb
from horizontal_toolbar import HorizontalToolbar


class MainToolbar(HorizontalToolbar):
    """An always visible toolbar for general actions with the state machine."""

    # enums describing reasons for path updates
    AUTO = 0
    USER = 1
    RESET = 2

    def __init__(self, main_window, *args, **kwargs):
        """Initialize the toolbar, its GUI elements and the callbacks."""
        HorizontalToolbar.__init__(self, *args, **kwargs)
        self.__graph_view_icon = geb.build_image(geb.IMG_DIR + 'graph_view.png')
        self.__tree_view_icon = geb.build_image(geb.IMG_DIR + 'tree_view.png')
        self.__graph_view = main_window.graph_view
        self.__tree_view = main_window.tree_view
        self.path_update = self.AUTO  # Was path combobox manually updated by user?

        # Create all GUI elements and add them to the toolbar
        self.toggle_view_button = geb.build_button(tooltip='Switch between graph and tree view',
                                                   icon=self.__graph_view_icon)
        self.toggle_view_button.connect('clicked', self.toggle_view)
        self.add(self.toggle_view_button)

        self.add(geb.build_label(geb.LABEL_SPACER + 'State: '))
        self.path_combo = geb.build_combo_box(main_window.tree_model, main_window.list_model, has_entry=True)
        self.add(self.path_combo)

        self.add_spacer()
        self.trigger_transition_button = geb.build_button(
            icon=geb.IMG_DIR + 'trigger_transition.png',
            tooltip='Trigger Transition')
        self.add(self.trigger_transition_button)

    def toggle_view(self, *args):
        def toggle(icon, to_show, to_hide):
            self.toggle_view_button.set_image(icon)
            to_show.show()
            to_hide.hide()

        if self.__graph_view.is_visible():
            toggle(self.__tree_view_icon, self.__tree_view, self.__graph_view)
        else:
            toggle(self.__graph_view_icon, self.__graph_view, self.__tree_view)

    def set_path(self, item, reason=USER):
        if reason != self.AUTO or self.path_update == self.AUTO:
            self.path_combo.set_active_iter(item)
            self.path_update = self.AUTO if reason == self.RESET else self.USER
