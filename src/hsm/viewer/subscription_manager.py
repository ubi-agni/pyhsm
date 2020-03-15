from threading import Thread

import rospy
import pyhsm_msgs.msg as msgs

from .. import introspection


class SubscriptionManager(object):
    """
    A handler serving as the connection between a viewer and servers its backend is listening to.
    Periodically updates the list of servers and subscribes to new ones with their correct
    callback functions.
    """

    def __init__(self, viewer):
        self._viewer = viewer
        self._update_cond = self._viewer.update_cond
        self._state_tree_model = self._viewer.state_tree_model

        self._structure_subs = {}
        self._status_subs = {}

        self._client = introspection.IntrospectionClient()
        self._update_servers_thread = Thread(target=self._update_server_loop)
        self._update_servers_thread.start()

    def __del__(self):
        self._update_servers_thread.join()
        super(SubscriptionManager, self).__del__(self)

    def _update_server_loop(self):
        """Periodically update the known HSM introspection servers."""
        while self._viewer.keep_running:
            # Update the server list
            server_names = self._client.get_servers()

            new_server_names = [sn for sn in server_names if sn not in self._structure_subs]
            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subscribe(server_name)


            # Remove old servers that are not in the list anymore.
            # TODO Should we maybe only do this after some timeout?
            inactive_server_names = [sn for sn in self._structure_subs if sn not in server_names]
            for server_name in inactive_server_names:
                self._unsubscribe(server_name)

            # Don't update the list too often
            with self._update_cond:
                self._update_cond.wait(1.0)

    def _structure_subscribe(self, server_name):
        """Subscribe to a new structure messaging server."""
        self._structure_subs[server_name] = rospy.Subscriber(
            server_name + introspection.STRUCTURE_TOPIC,
            msgs.HsmStructure,
            callback=self._build_and_status_sub,
            callback_args=server_name,
            queue_size=50,
        )

    def _build_and_status_sub(self, msg, server_name):
        """
        Build the tree as given by the ``HsmStructure`` message and subscribe to the server's
        status messages.
        """
        if not self._viewer.keep_running:
            return

        rospy.logdebug("STRUCTURE MSG WITH PREFIX: " + msg.prefix)

        with self._update_cond:
            local_root = self._state_tree_model.build_from_structure_msg(msg, server_name)
            self._viewer.hsms[server_name] = local_root

            # TODO We could also only `expand_row` for each path in msg.states
            self._viewer.tree_view.expand_all()

            self._viewer.needs_graph_update = True
            self._viewer.needs_tree_update = True
            self._update_cond.notify_all()

        # Once we have the HSM's nodes, we can update its status
        self._status_subscribe(server_name)

    def _status_subscribe(self, server_name):
        """Subscribe to a new status messaging server."""
        self._status_subs[server_name] = rospy.Subscriber(
            server_name + introspection.STATUS_TOPIC,
            msgs.HsmCurrentState,
            callback=self._update_tree,
            callback_args=server_name,
            queue_size=50,
        )

    def _update_tree(self, msg, server_name):
        """Update the tree using the given ``HsmCurrentState`` message."""
        if not self._viewer.keep_running:
            return

        rospy.logdebug("STATUS MSG: " + msg.path)

        with self._update_cond:
            needs_update = self._state_tree_model.update_active_from_current_state_msg(
                msg, server_name)

            if needs_update:
                self._viewer.needs_graph_update = True
                self._viewer.needs_tree_update = True
                self._update_cond.notify_all()
