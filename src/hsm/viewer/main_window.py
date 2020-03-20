import threading
import rospy

from gtk_wrap import Gtk, GObject
from gui_element_builder import build_box
from graph_view import GraphView
from main_toolbar import MainToolbar
from state_tree_model import StateTreeModel
from ..introspection import *
from tree_view import TreeView
from pyhsm_msgs.msg import HsmStructure, HsmCurrentState, HsmTransition
from std_msgs.msg import String


class Subscription(object):
    def __init__(self, structure_sub):
        self.structure_sub = structure_sub
        self.status_sub = None
        self.roots = set()


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

        self._subs = {}  # map from server_name to Subscription
        self._update_servers_thread = threading.Thread(target=self._update_servers)

        self._update_cond = threading.Condition()
        self._update_graph = False
        self._keep_running = True

        # Backend
        self.tree_model = StateTreeModel()  # Tree store representing the state hierarchy
        self.list_model = Gtk.ListStore(str)  # Flat model of the tree store

        # Frontend
        self.__setup_gui_elements()
        self.show_all()
        self.main_toolbar.toggle_view()  # start with tree view for now

        # Start serving: retrieve servers, subscribe to structure and state msgs
        self.connect('destroy', self._stop)
        self._update_servers_thread.start()

    def _stop(self, *args):
        with self._update_cond:
            self._keep_running = False  # signal shutdown
            self._update_cond.notify_all()
        self._update_servers_thread.join()

    def __setup_gui_elements(self):
        """Create all GUI elements and add them to the window."""
        root_vbox = self.__add_to(self, build_box(4))  # top level vertical box

        # Prepare graph and tree view
        self.graph_view = GraphView(self.tree_model)  # Graph view including its toolbar
        self.tree_view = TreeView(self.tree_model)  # Tree view of all paths
        self.main_toolbar = MainToolbar(self)  # main toolbar

        # Add elements in correct order
        self.__add_to(root_vbox, self.main_toolbar)
        self.__add_to(root_vbox, self.graph_view, expand=True)
        self.__add_to(root_vbox, self.tree_view, expand=True)

        ### Connect signals
        # link tree view selection to path combobox
        self.main_toolbar.path_combo.connect('changed', self.on_path_combo_changed)
        selection = self.tree_view.get_selection()
        selection.connect('changed', self.on_tree_selection_changed)

        # trigger state transition on double-click in tree view or on trigger_transition_button
        self.tree_view.connect('row-activated', self.on_trigger_transition)
        self.main_toolbar.trigger_transition_button.connect('clicked', self.on_trigger_transition)

    @staticmethod
    def __add_to(parent, child, expand=False):
        """Add the given child to the given parent. If the parent is a ``Gtk.Box``,
           the ``expand`` flag controls whether the added item will fill its space."""
        if isinstance(parent, Gtk.Box):
            parent.pack_start(child, expand, expand, 0)
        else:
            parent.add(child)
        return child

    ### GUI event handlers

    def on_path_combo_changed(self, combo):
        model = combo.get_model()
        item = combo.get_active_iter()
        if item is None:  # content was changed by typing
            # try to find existing item from text
            entry = combo.get_child()
            text = entry and entry.get_text()
            item = text and model.find_node(text)
            item and combo.set_active_iter(item)
        # enable trigger transition button depending on selection
        self.main_toolbar.trigger_transition_button.set_sensitive(item and model[item][model.ENABLED] or False)

    def on_tree_selection_changed(self, selection):
        model, item = selection.get_selected()
        if item is not None:
            self.main_toolbar.set_path(item)

    def on_trigger_transition(self, source, *args):
        if isinstance(source, Gtk.Button):
            item = self.main_toolbar.path_combo.get_active_iter()
        elif isinstance(source, Gtk.TreeView):
            path = args[0]
            item = source.get_model().get_iter(path)
        if item is not None:
            root = self.tree_model.root_state(item)
            path = self.tree_model.path(item)[len(root.prefix):]  # strip prefix from path
            root.transition_publisher.publish(String(path))

    ### ROS connection / structure updates

    def _update_servers(self):
        """Periodically update the known HSM introspection servers."""
        with self._update_cond:
            while self._keep_running and not rospy.is_shutdown():
                # Update the server list
                server_names = IntrospectionClient.get_servers()

                new_server_names = [sn for sn in server_names if sn not in self._subs.keys()]
                # Subscribe to new servers
                for server_name in new_server_names:
                    # process message synchronously in GUI thread
                    callback = lambda msg, server_name: GObject.idle_add(self._process_structure_msg, msg, server_name)
                    s = Subscription(IntrospectionClient.subscribe(server_name, callback, callback_args=server_name))
                    self._subs[server_name] = s

                # Don't update the list too often
                self._update_cond.wait(1.0)

    def _remove_server(self, server_name, prefix):
        """Handle removal of server_name"""
        with self._update_cond:
            server = self._subs[server_name]
            server.status_sub.unregister()

            # mark all root states as disabled
            for root_state in server.roots:
                root = self.tree_model.find_node(root_state.path)
                root and self.tree_model.enable(root, False, recursive=True)

            # remove all root states from model after 10s
            def remove_states(states):
                for state in states:
                    root = self.tree_model.find_node(state.path)
                    self.tree_model.remove(root)
                self._update_list_model()

            GObject.timeout_add(10000, remove_states, server.roots)
            server.roots = set()  # clear set

    def _process_structure_msg(self, msg, server_name):
        """Build the tree as given by the ``HsmStructure`` message and subscribe to the server's status messages."""
        rospy.logdebug("STRUCTURE MSG WITH PREFIX: " + msg.prefix)
        if rospy.is_shutdown():
            return

        if not msg.states:
            self._remove_server(server_name, msg.prefix)
            return

        with self._update_cond:
            server = self._subs[server_name]
            root = self.tree_model.update_tree(msg, server_name)
            root_state = self.tree_model.root_state(root)
            server.roots.add(root_state)
            self._update_list_model()

            # Expand new states
            path = self.tree_model.get_path(root)
            self.tree_view.expand_to_path(path)  # expand tree to make root visible
            self.tree_view.expand_row(path, True)  # recursively expand root and its children

            self._update_graph = True
            self._update_cond.notify_all()

        # Once we have the HSM's nodes, we can update its status
        server.status_sub = rospy.Subscriber(
            server_name + STATUS_TOPIC, HsmCurrentState,
            # process message synchronously in GUI thread
            callback=lambda msg: GObject.idle_add(self._process_status_msg, msg, root_state),
            queue_size=50)

    def _process_status_msg(self, msg, root_state):
        """Update the tree using the given ``HsmCurrentState`` message."""
        if rospy.is_shutdown():
            return

        rospy.logdebug("STATUS MSG: " + msg.path)

        with self._update_cond:
            self._update_graph = self.tree_model.update_current_state(msg, root_state)
            if self._update_graph:
                toolbar = self.main_toolbar
                item = self.tree_model.find_node(root_state.prefix + msg.path)
                if item is not None and toolbar.path_update == MainToolbar.AUTO:
                    toolbar.set_path(item)
                    toolbar.path_update = MainToolbar.AUTO  # keep AUTO
                self._update_cond.notify_all()

    def _update_list_model(self):
        tree = self.tree_model
        column = tree.PATH
        list = self.list_model
        list.clear()
        def traverse(parent=None):
            item = tree.iter_children(parent)
            while item:
                list.append(row=[tree[item][column]])
                traverse(item)
                item = tree.iter_next(item)
        traverse()
