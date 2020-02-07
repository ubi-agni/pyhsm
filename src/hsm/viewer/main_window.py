import threading

from gtk_wrap import Gtk
import gui_element_builder as geb
import graph_view as gv
import main_toolbar as mt


class MainWindow(Gtk.Window):
    """Main window of the HSM viewer containing the toolbar, graph and tree view."""

    def __init__(self, width, height):
        Gtk.Window.__init__(self, title='HSM Viewer')
        self.set_border_width(5)
        self.set_default_size(width, height)

        self.__update_cond = threading.Condition()
        self.__keep_running = True

        # Backend
        self.container_tree_model = geb.build_tree_model()
        """Tree store containing the containers."""

        # Frontend
        self.__setup_gui_elements()
        self.show_all()
        # Revert the ``no_show_all`` behavior so the tree is hidden in case we call ``hide_all``.
        self.tree_view.set_no_show_all(False)

    def __setup_gui_elements(self):
        """Create all GUI elements and add them to the window."""
        root_vbox = self.__add_to(self, geb.build_box(4))
        """The top level vertical boxing element."""

        # Prepare graph and tree view
        self.graph_view = gv.GraphView(self.container_tree_model)
        """Graph view including its toolbar."""
        self.tree_view = geb.build_tree_view(self.container_tree_model)
        """Tree view of all paths. Enables some interaction such as double clicking."""
        self.tree_view.set_no_show_all(True)

        self.main_toolbar = mt.MainToolbar(self)
        """Always visible toolbar."""

        # Add elements in correct order
        self.__add_to(root_vbox, self.main_toolbar)
        self.__add_to(root_vbox, self.graph_view, expand=True)
        self.__add_to(root_vbox, self.tree_view, expand=True)

        self.statusbar = self.__add_to(root_vbox, geb.build_statusbar())
        """Status bar at the bottom of the window."""

    def __del__(self):
        """Signal that the viewer is going to shut down."""
        with self.__update_cond:
            self.__keep_running = False
            self.__update_cond.notify_all()
        if hasattr(Gtk.Window, '__del__'):
            Gtk.Window.__del__(self)

    @staticmethod
    def __add_to(parent, child, expand=False):
        """
        Add the given child to the given parent. If the parent is a ``Gtk.Box``, the
        ``expand`` flag controls whether the added item will fill its space.
        """
        if isinstance(parent, Gtk.Box):
            parent.pack_start(child, expand, expand, 0)
        else:
            parent.add(child)
        return child
