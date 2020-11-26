import threading
import rospy
import os

from . import *
from state_tree_model import StateTreeModel
from graph_view import GraphView
from ..introspection import *
from pyhsm_msgs.msg import HsmStructure, HsmCurrentState, HsmTransition
from std_msgs.msg import String


class Subscription(object):
    def __init__(self, structure_sub):
        self.structure_sub = structure_sub
        self.status_sub = None
        self.roots = set()


class Gui(object):
    # enums describing path_combo's update mode
    AUTO = 0
    USER = 1

    def __init__(self, width, height):
        self._subs = {}  # map from server_name to Subscription
        self._update_servers_thread = threading.Thread(target=self._update_servers)

        self._update_cond = threading.Condition()
        self._keep_running = True

        # data model
        self.tree_model = StateTreeModel()  # Tree store representing the state hierarchy
        self.list_model = Gtk.ListStore(str)  # Flat model of the tree store

        # setup gui
        self.path_update_mode = self.AUTO
        builder = Gtk.Builder()
        builder.add_from_file(os.path.join(os.path.dirname(__file__), 'gui', 'gui.glade'))

        # fetch objects from builder
        self.main = builder.get_object('main')
        self.tree_view = builder.get_object('tree_view')
        self._configure_tree_view(self.tree_model)
        self.graph_view = GraphView(builder, self.tree_model)
        self.graph_view.update()

        self.stack = builder.get_object('stack')
        setattr(self.stack, 'toggle_view_button', builder.get_object('toggle_view_button'))
        for name in ['tree', 'graph']:  # store icons in child widgets with these names
            setattr(self.stack.get_child_by_name(name), 'icon', builder.get_object(name + '_view_icon'))
        self.toggle_view(view='graph')

        self.trigger_transition_button = builder.get_object('trigger_transition_button')
        self.path_combo = builder.get_object('path_combo')
        self.filter_combo = builder.get_object('filter_combo')

        # configure models
        for combobox in [self.path_combo, self.filter_combo]:
            self._configure_combo_box(combobox, self.tree_model, self.list_model)

        self.main.set_default_size(width, height)
        self._connect_signals(builder)
        self.main.show_all()

        # Start serving: retrieve servers, subscribe to structure and state msgs
        self._update_servers_thread.start()

    def _quit(self, *args):
        with self._update_cond:
            self._keep_running = False  # signal shutdown
            self._update_cond.notify_all()
        self._update_servers_thread.join()
        Gtk.main_quit()

    def _configure_tree_view(self, model):
        self.tree_view.set_model(model)

        # Configure rendering: display label as text, use weight
        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn('Path', renderer, text=model.LABEL, sensitive=model.ENABLED, weight=model.WEIGHT)
        column.set_sort_column_id(model.LABEL)  # sort by label
        self.tree_view.append_column(column)

        # Disable selection of disabled rows
        selection = self.tree_view.get_selection()
        def select_function(selection, model, path, *args):
            return model[path][model.ENABLED]
        selection.set_select_function(select_function)

    @staticmethod
    def _configure_combo_box(combo, model, completion_model):
        combo.set_model(model)
        combo.set_entry_text_column(model.PATH)  # required to display full path in text field

        # configure combobox style to be list-like (instead of menu-like)
        css = Gtk.CssProvider()
        css.load_from_data("* { -GtkComboBox-appears-as-list: True; }")
        combo.get_style_context().add_provider(css, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

        # configure (short) labels to be displayed in popup view
        combo.clear()  # remove (full) path renderer added by set_entry_text_column()
        renderer = Gtk.CellRendererText()
        combo.pack_start(renderer, True)
        combo.add_attribute(renderer, 'text', model.LABEL)
        combo.add_attribute(renderer, 'weight', model.WEIGHT)
        combo.add_attribute(renderer, 'sensitive', model.ENABLED)

        # configure completion for text field (entry)
        if combo.get_has_entry() and completion_model is not None:
            completion = Gtk.EntryCompletion(model=completion_model, minimum_key_length=2, inline_completion=True)
            completion.set_text_column(0)  # configures data column and renderer
            entry = combo.get_child()
            entry.set_completion(completion)

    def _connect_signals(self, builder):
        builder.connect_signals(self)
        self.graph_view.dot_widget.connect('clicked', self.on_graph_selection_changed)
        self.graph_view.dot_widget.connect('activated', self.on_graph_activated)

    def root_state_from_id(self, id):
        """Retrieve root state from given graph id"""
        for sub in self._subs.itervalues():
            for root in sub.roots:
                if id.startswith(root.server_name + ':' + root.prefix):
                    return root

    def item_from_id(self, id):
        """Retrieve tree item from given graph id"""
        root_state = self.root_state_from_id(id)
        if root_state is None:  # if the HSM already terminated, the root_state will be None
            return None
        root = self.tree_model.root_from_root_state(root_state)
        strip = len(self.graph_view.id(root_state)) + 1  # strip off root state's id + slash
        return root and self.tree_model.find_node(id[strip:], parent=root)

    ### GUI event handlers

    def toggle_view(self, *args, **kwargs):
        stack = self.stack
        view = kwargs.get('view', 'graph' if stack.get_visible_child_name() == 'tree' else 'tree')
        stack.set_visible_child_name(view)
        view = stack.get_visible_child()
        stack.toggle_view_button.set_icon_widget(view.icon)

    def on_path_combo_changed(self, combo):
        model = combo.get_model()
        item = combo.get_active_iter()
        self.path_update_mode = self.USER
        if item is None:  # content was changed by typing
            # try to find existing item from text
            entry = combo.get_child()
            text = entry and entry.get_text()
            if not text:  # empty text resets to AUTO mode
                self.path_update_mode = self.AUTO
            else:
                item = model.find_node(text)  # try to find item
            item and combo.set_active_iter(item)
        else:
            self.graph_view.set_selected(item)
        # enable trigger transition button depending on selection
        self.trigger_transition_button.set_sensitive(item and model[item][model.ENABLED] or False)

    def on_graph_selection_changed(self, widget, url, event):
        item = self.item_from_id(url)
        item and self.tree_view.get_selection().select_path(self.tree_model.get_path(item))

    def on_graph_activated(self, widget, url, event):
        item = self.item_from_id(url)
        item and self.on_trigger_transition(widget, item)

    def on_tree_selection_changed(self, selection):
        model, item = selection.get_selected()
        if item is not None:
            self.path_combo.set_active_iter(item)

    def on_trigger_transition(self, source, *args):
        if source == self.trigger_transition_button:
            item = self.path_combo.get_active_iter()
        elif source is self.tree_view:
            path = args[0]
            item = source.get_model().get_iter(path)
        elif source is self.graph_view.dot_widget:
            item = args[0]
        else:
            raise TypeError('Unknown source: ' + str(source))

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
            def remove_states(roots):
                for root_state in roots:
                    root = self.tree_model.find_node(root_state.path)
                    # remove root state and all its orphaned parents
                    while root:
                        parent = self.tree_model.iter_parent(root)
                        self._invalidate_comboboxes(self.tree_model.path(root))
                        self.tree_model.remove(root)
                        if self.tree_model.iter_children(parent): # no more children?
                            root = None # stop loop
                        else:
                            root = parent # traverse tree upwards
                self._update_list_model()

            GObject.timeout_add(10000, remove_states, server.roots)
            server.roots = set()  # clear set

    def _invalidate_comboboxes(self, state_path):
        """Invalidate any combobox showing state_path as its current item"""
        for combo in [self.path_combo, self.filter_combo]:
            item = combo.get_active_iter()
            entry = combo.get_child()
            text = (item and combo.get_model().path(item)) or (entry and entry.get_text())
            if state_path == text or text.startswith(state_path + '/'):
                combo.set_active_iter(None)
                entry and entry.set_text('')

    def _process_structure_msg(self, msg, server_name):
        """Build the tree as given by the ``HsmStructure`` message and subscribe to the server's status messages."""
        rospy.logdebug("STRUCTURE MSG WITH PREFIX: " + msg.prefix)
        if rospy.is_shutdown():
            return

        if not msg.states:  # empty states list signals that server has finished
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

            self.graph_view.update()
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
            changed = self.tree_model.update_current_state(msg, root_state)
            if changed and self.path_update_mode == self.AUTO:
                # update current state in path combobox
                item = self.tree_model.find_node(root_state.prefix + msg.path)
                if item is not None:
                    self.path_combo.set_active_iter(item)
                    self.path_update_mode = self.AUTO  # keep AUTO mode
            # TODO: use signals
            self.graph_view.update_styles(self.tree_model)

    def _update_list_model(self):
        tree = self.tree_model
        column = tree.PATH
        flat = self.list_model
        flat.clear()
        def traverse(parent=None):
            item = tree.iter_children(parent)
            while item:
                flat.append(row=[tree[item][column]])
                traverse(item)
                item = tree.iter_next(item)
        traverse()
