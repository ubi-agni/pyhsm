import threading

from gtk_wrap import Gtk
from gui_element_builder import build_box
from graph_view import GraphView
from main_toolbar import MainToolbar
from state_tree_model import StateTreeModel
from subscription_manager import SubscriptionManager
from tree_view import TreeView


class MainWindow(Gtk.Window):
    """
    Main window of the HSM viewer containing the toolbar, graph and tree view.

    Also handles some backend activities such as initializing the model and starting threads.
    """

    def __init__(self, width, height):
        """Initialize the main window with its backend models as well as its GUI elements."""
        Gtk.Window.__init__(self, title='HSM Viewer')
        self.set_border_width(5)
        self.set_default_size(width, height)

        self.update_cond = threading.Condition()
        self.needs_graph_update = False

        # Backend
        self.state_tree_model = StateTreeModel()  # Tree store containing the containers

        # Frontend
        self.__setup_gui_elements()
        self.show_all()
        self.main_toolbar.toggle_view()  # start with tree view for now

        # Start serving: retrieve servers, subscribe to structure and state msgs
        self.subscription_manager = SubscriptionManager(self)

    def __setup_gui_elements(self):
        """Create all GUI elements and add them to the window."""
        root_vbox = self.__add_to(self, build_box(4))  # top level vertical box

        # Prepare graph and tree view
        self.graph_view = GraphView(self.state_tree_model)  # Graph view including its toolbar
        self.tree_view = TreeView(self.state_tree_model)  # Tree view of all paths
        self.main_toolbar = MainToolbar(self)  # main toolbar

        # Add elements in correct order
        self.__add_to(root_vbox, self.main_toolbar)
        self.__add_to(root_vbox, self.graph_view, expand=True)
        self.__add_to(root_vbox, self.tree_view, expand=True)

    @staticmethod
    def __add_to(parent, child, expand=False):
        """Add the given child to the given parent. If the parent is a ``Gtk.Box``,
           the ``expand`` flag controls whether the added item will fill its space."""
        if isinstance(parent, Gtk.Box):
            parent.pack_start(child, expand, expand, 0)
        else:
            parent.add(child)
        return child
