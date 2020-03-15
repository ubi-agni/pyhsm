import gui_element_builder as geb
from horizontal_toolbar import HorizontalToolbar


class MainToolbar(HorizontalToolbar):
    """An always visible toolbar for general actions with the state machine."""

    def __init__(self, main_window, *args, **kwargs):
        """Initialize the toolbar, its GUI elements and the callbacks."""
        HorizontalToolbar.__init__(self, *args, **kwargs)
        self.__graph_view_icon = geb.build_image(geb.IMG_DIR + 'graph_view.png')
        self.__tree_view_icon = geb.build_image(geb.IMG_DIR + 'tree_view.png')
        self.__showing_graph_view = True

        self.__setup_gui_elements(main_window.state_tree_model)
        self.__connect_callbacks(main_window)

    def __setup_gui_elements(self, model):
        """Create all GUI elements and add them to the toolbar."""
        self.toggle_view_button = geb.build_button(icon=self.__graph_view_icon,
                                                   tooltip='Switch between graph and tree view')
        self.add(self.toggle_view_button)

        self.add(geb.build_label(geb.LABEL_SPACER + 'Current Path: '))
        self.path_combo = geb.build_combo_box(model)
        self.add(self.path_combo)

        self.add_spacer()
        self.trigger_transition_button = geb.build_button(
            icon_path=geb.IMG_DIR + 'trigger_transition.png',
            tooltip='Trigger Transition')
        self.add(self.trigger_transition_button)

    def __connect_callbacks(self, main_window):
        """Connect all callbacks to the GUI elements."""
        self.toggle_view_button.connect('clicked', self.__toggle_view,
                                        main_window.graph_view, main_window.tree_view)
        # FIXME

    def __toggle_view(self, button, graph_view, tree_view):
        if self.__showing_graph_view:
            self.__toggle_view_action(button, self.__tree_view_icon, tree_view, graph_view, False)
        else:
            self.__toggle_view_action(button, self.__graph_view_icon, graph_view, tree_view, True)

    def __toggle_view_action(self, button, icon, to_show, to_hide, was_showing_tree_view):
        button.set_image(icon)
        to_show.show()
        to_hide.hide()
        self.__showing_graph_view = was_showing_tree_view
