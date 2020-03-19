from threading import Thread
from gtk_wrap import GObject

import rospy
import pyhsm_msgs.msg as msgs

from .. import introspection


class SubscriptionManager(object):
    """Manage server connections of a viewer.

    Periodically updates the list of servers and subscribes to new ones.
    """

    def __init__(self, viewer):
        self._viewer = viewer
        self._update_cond = self._viewer.update_cond
        self._state_tree_model = self._viewer.state_tree_model

        self._structure_subs = {}
        self._status_subs = {}

        self._client = introspection.IntrospectionClient()
        self._update_servers_thread = Thread(target=self._update_servers)
        self._update_servers_thread.start()

    def __del__(self):
        self._update_servers_thread.join()
        super(SubscriptionManager, self).__del__(self)

    def _update_servers(self):
        """Periodically update the known HSM introspection servers."""
        while not rospy.is_shutdown():
            # Update the server list
            server_names = self._client.get_servers()

            new_server_names = [sn for sn in server_names if sn not in self._structure_subs]
            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subscribe(server_name)

            # Remove old servers that are not in the list anymore.
            # TODO Should we maybe only do this after some timeout?
            removed_server_names = [sn for sn in self._structure_subs if sn not in server_names]
            for server_name in removed_server_names:
                self._unsubscribe(server_name)

            # Don't update the list too often
            with self._update_cond:
                self._update_cond.wait(1.0)

    def _structure_subscribe(self, server_name):
        """Subscribe to a new structure messaging server."""
        self._structure_subs[server_name] = rospy.Subscriber(
            server_name + introspection.STRUCTURE_TOPIC,
            msgs.HsmStructure,
            # process message synchronously in GUI thread
            callback=lambda msg: GObject.idle_add(self._process_structure_msg, msg, server_name),
            queue_size=50,
        )

    def _process_structure_msg(self, msg, server_name):
        """Build the tree as given by the ``HsmStructure`` message and subscribe to the server's status messages."""
        if rospy.is_shutdown():
            return

        rospy.logdebug("STRUCTURE MSG WITH PREFIX: " + msg.prefix)

        with self._update_cond:
            gui = self._viewer
            root = self._state_tree_model.update_tree(msg, server_name)

            # Expand new states
            path = self._state_tree_model.get_path(root)
            gui.tree_view.expand_to_path(path)  # expand tree to make root visible
            gui.tree_view.expand_row(path, True)  # recursively expand root and its children

            gui.needs_graph_update = True
            self._update_cond.notify_all()

        # Once we have the HSM's nodes, we can update its status
        self._status_subscribe(self._state_tree_model.state(root))

    def _status_subscribe(self, root):
        """Subscribe to a new status messaging server."""
        self._status_subs[root.server_name] = rospy.Subscriber(
            root.server_name + introspection.STATUS_TOPIC,
            msgs.HsmCurrentState,
            # process message synchronously in GUI thread
            callback=lambda msg: GObject.idle_add(self._process_status_msg, msg, root),
            queue_size=50,
        )

    def _process_status_msg(self, msg, root):
        """Update the tree using the given ``HsmCurrentState`` message."""
        if rospy.is_shutdown():
            return

        rospy.logdebug("STATUS MSG: " + msg.path)

        with self._update_cond:
            self._viewer.needs_graph_update = self._state_tree_model.update_current_state(msg, root)
            if self._viewer.needs_graph_update:
                self._update_cond.notify_all()

    def _unsubscribe(self, server_name):
        # FIXME Implement this: Remove callbacks and update tree with removed nodes.
        pass
