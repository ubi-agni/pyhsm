#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#   * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren 

from __future__ import print_function

import rospy
import rospkg

from std_msgs.msg import String

import sys
import os
import threading
import pickle
import time

import wx
import wx.richtext
import textwrap

## this import system (or ros-released) xdot
# import xdot
## need to import currnt package, but not to load this file
# http://stackoverflow.com/questions/6031584/importing-from-builtin-library-when-module-with-same-name-exists
def import_non_local(name, custom_name=None):
    import imp, sys

    custom_name = custom_name or name

    path = filter(lambda x: x != os.path.dirname(os.path.abspath(__file__)), sys.path)
    f, pathname, desc = imp.find_module(name, path)

    module = imp.load_module(custom_name, f, pathname, desc)
    if f:
        f.close()

    return module

smach_viewer = import_non_local('smach_viewer')
from smach_viewer import xdot
##
import smach
import pyhsm_msgs.msg as msgs

import hsm.introspection

### Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ';\n'.join(attrs_strs)+';\n'

def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ' ['+(', '.join(attrs_strs))+']'

def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path

def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]

def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [int(color_str[i:i+2],16)/255.0    for i in range(1,len(color_str),2)]
    return color_tuple

class ContainerNode(object):
    """
    This class represents a given container in a running SMACH system. 

    Its primary use is to generate dotcode for a SMACH container. It has
    methods for responding to structure and status messages from a SMACH
    introspection server, as well as methods for updating the styles of a 
    graph once it's been drawn.
    """

    def __init__(self, server_name, msg, prefix, children):
        # Store path info
        self._server_name = server_name
        if prefix and prefix[-1] != '/':
            prefix += '/'
        self._path = prefix + msg.path
        splitpath = msg.path.split('/')
        self._label = splitpath[-1]
        self._dir = '/'.join(splitpath[0:-1])

        self._children = children

        # TODO These seem to be transition targets
        # self._container_outcomes = msg.container_outcomes

        # Status
        # TODO not as list
        self._initial_states = [msg.initial]
        self._is_active = False
        # Labels of active children
        self._active_states = []
        self._last_active_states = []

    @property
    def is_active(self):
        return self._is_active

    @is_active.setter
    def is_active(self, is_active):
        """Update this node's status to be ``set_active``."""
        if not is_active:
            self.active_states = []
        else:
            self._is_active = True

    @property
    def active_states(self):
        return self._active_states

    @active_states.setter
    def active_states(self, states):
        self._active_states = states
        self._is_active = bool(states)

    def get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.
        
        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            #attrs['style'] = 'filled,rounded'
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'
        #attrs['rank'] = 'max'

        #,'succeeded','aborted','preempted'attrs['label'] = self._label
        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL':self._path,
                'shape':'plaintext',
                'color':'gray',
                'fontsize':'18',
                'fontweight':'18',
                'rank':'min',
                'height':'0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path,'__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path,'__outcomes__'])
            outcomes_attrs = {
                    'style':'rounded,filled',
                    'rank':'sink',
                    'color':'#FFFFFFFF',#'#871C34',
                    'fillcolor':'#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            # TODO
            # for outcome_label in self._container_outcomes:
            #     outcome_path = ':'.join([self._path,outcome_label])
            #     outcome_attrs = {
            #             'shape':'box',
            #             'height':'0.3',
            #             'style':'filled,rounded',
            #             'fontsize':'12',
            #             'fillcolor':'#FE464f',#'#EDC2CC',
            #             'color':'#780006',#'#EBAEBB',
            #             'fontcolor':'#780006',#'#EBAEBB',
            #             'label':'',
            #             'xlabel':'\\n'.join(label_wrapper.wrap(outcome_label)),
            #             'URL':':'.join([self._path,outcome_label])
            #             }
            #     dotstr += '"%s" %s;\n' % (outcome_path,attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_attrs = {
                        'style':'filled,setlinewidth(2)',
                        'color':'#000000FF',
                        'fillcolor':'#FFFFFF00'
                        }

                child_path = '/'.join([self._path,child_label])
                # Generate dotcode for children
                if child_path in containers:
                    child_attrs['style'] += ',rounded'

                    dotstr += containers[child_path].get_dotcode(
                            selected_paths,
                            closed_paths,
                            depth+1, max_depth,
                            containers,
                            show_all,
                            label_wrapper,
                            child_attrs)
                else:
                    child_attrs['label'] = '\\n'.join(label_wrapper.wrap(child_label))
                    child_attrs['URL'] = child_path
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = []

            # Add edge from container label to initial state
            internal_edges += [('','__proxy__',initial_child) for initial_child in self._initial_states]

            has_explicit_transitions = []
            for (outcome_label,from_label,to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label,from_label,to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL':':'.join([from_path,outcome_label,to_path]),
                            'fontsize':'12',
                            'label':'',
                            'xlabel':'\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    #if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % ( from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path,'__proxy__'])
                            from_key = '"%s"' % ( from_path )

                    to_key = ''
                    # TODO
                    # if to_label in self._container_outcomes:
                    if False and to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path,to_label)
                        edge_attrs['color'] = '#00000055'# '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path,'__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr

    def set_styles(self, selected_paths, depth, max_depth, items, subgraph_shapes, containers):
        """Update the styles for a list of containers without regenerating the dotcode.

        This function is called recursively to update an entire tree.
        
        @param selected_paths: A list of paths to nodes that are currently selected.
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to traverse into the tree
        @param items: A dict of all the graph items, keyed by url
        @param subgraph_shapes: A dictionary of shapes from the rendering engine
        @param containers: A dict of all the containers
        """

        # Color root container
        """
        if depth == 0:
            container_shapes = subgraph_shapes['cluster_'+self._path]
            container_color = (0,0,0,0)
            container_fillcolor = (0,0,0,0)

            for shape in container_shapes:
                shape.pen.color = container_color
                shape.pen.fillcolor = container_fillcolor
                """

        # Color shapes for outcomes

        # Color children
        if max_depth == -1 or depth <= max_depth:
            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path,child_label])

                child_color = [0.5,0.5,0.5,1]
                child_fillcolor = [1,1,1,1]
                child_linewidth = 2

                active_color = hex2t('#5C7600FF')
                active_fillcolor = hex2t('#C0F700FF')

                initial_color = hex2t('#000000FF')
                initial_fillcolor = hex2t('#FFFFFFFF')

                if child_label in self._active_states:
                    # Check if the child is active
                    child_color = active_color
                    child_fillcolor = active_fillcolor
                    child_linewidth = 5
                elif child_label in self._initial_states:
                    # Initial style
                    #child_fillcolor = initial_fillcolor
                    child_color = initial_color
                    child_linewidth = 2

                # Check if the child is selected
                if child_path in selected_paths:
                    child_color = hex2t('#FB000DFF')

                # Generate dotcode for child containers 
                if child_path in containers:
                    subgraph_id = 'cluster_'+child_path
                    if subgraph_id in subgraph_shapes:
                        if child_label in self._active_states:
                            child_fillcolor[3] = 0.25
                        elif 0 and child_label in self._initial_states:
                            child_fillcolor[3] = 0.25
                        else:
                            if max_depth > 0:
                                v = 1.0-0.25*((depth+1)/float(max_depth))
                            else:
                                v = 0.85
                            child_fillcolor = [v,v,v,1.0]

                        
                        for shape in subgraph_shapes['cluster_'+child_path]:
                            pen = shape.pen
                            if len(pen.color) > 3:
                                pen_color_opacity = pen.color[3]
                                if pen_color_opacity < 0.01:
                                    pen_color_opacity = 0
                            else:
                                pen_color_opacity = 0.5
                            shape.pen.color = child_color[0:3]+[pen_color_opacity]
                            shape.pen.fillcolor = [child_fillcolor[i] for i in range(min(3,len(pen.fillcolor)))]
                            shape.pen.linewidth = child_linewidth

                        # Recurse on this child
                        containers[child_path].set_styles(
                                selected_paths,
                                depth+1, max_depth,
                                items,
                                subgraph_shapes,
                                containers)
                else:
                    if child_path in items:
                        for shape in items[child_path].shapes:
                            if not isinstance(shape,xdot.xdot.TextShape):
                                shape.pen.color = child_color
                                shape.pen.fillcolor = child_fillcolor
                                shape.pen.linewidth = child_linewidth
                    else:
                        #print child_path+" NOT IN "+str(items.keys())
                        pass

class SmachViewerFrame(wx.Frame):
    """
    This class provides a GUI application for viewing SMACH plans.
    """
    def __init__(self):
        wx.Frame.__init__(self, None, -1, "Smach Viewer", size=(720,480))

        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._tree_nodes = {}
        self._update_cond = threading.Condition()
        self._graph_update_complete = False
        self._tree_update_complete = False
        self._needs_refresh = True
        self.dotstr = ''

        # Full path to active node
        self._active_path = ''

        vbox = wx.BoxSizer(wx.VERTICAL)


        # Create Splitter
        self.content_splitter = wx.SplitterWindow(self, -1,style = wx.SP_LIVE_UPDATE)
        self.content_splitter.SetMinimumPaneSize(24)
        self.content_splitter.SetSashGravity(0.85)


        # Create viewer pane
        viewer = wx.Panel(self.content_splitter,-1)

        # Create smach viewer 
        nb = wx.Notebook(viewer,-1,style=wx.NB_TOP | wx.WANTS_CHARS)
        viewer_box = wx.BoxSizer()
        viewer_box.Add(nb,1,wx.EXPAND | wx.ALL, 4)
        viewer.SetSizer(viewer_box)

        # Create graph view
        graph_view = wx.Panel(nb,-1)
        gv_vbox = wx.BoxSizer(wx.VERTICAL)
        graph_view.SetSizer(gv_vbox)

        # Construct toolbar
        toolbar = wx.ToolBar(graph_view, -1)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"Path: "))

        # Path list
        self.path_combo = wx.ComboBox(toolbar, -1, style=wx.CB_DROPDOWN)
        self.path_combo .Bind(wx.EVT_COMBOBOX, self.set_path)
        self.path_combo.Append('/')
        self.path_combo.SetValue('/')
        toolbar.AddControl(self.path_combo)

        # Depth spinner
        self.depth_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=-1,
                max=1337,
                initial=-1)
        self.depth_spinner.Bind(wx.EVT_SPINCTRL,self.set_depth)
        self._max_depth = -1
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Depth: "))
        toolbar.AddControl(self.depth_spinner)

        # Label width spinner
        self.width_spinner = wx.SpinCtrl(toolbar, -1,
                size=wx.Size(50,-1),
                min=1,
                max=1337,
                initial=40)
        self.width_spinner.Bind(wx.EVT_SPINCTRL,self.set_label_width)
        self._label_wrapper = textwrap.TextWrapper(40,break_long_words=True)
        toolbar.AddControl(wx.StaticText(toolbar,-1,"    Label Width: "))
        toolbar.AddControl(self.width_spinner)

        # Implicit transition display
        toggle_all = wx.ToggleButton(toolbar,-1,'Show Implicit')
        toggle_all.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_all_transitions)
        self._show_all_transitions = False

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddControl(toggle_all)

        toggle_auto_focus = wx.ToggleButton(toolbar, -1, 'Auto Focus')
        toggle_auto_focus.Bind(wx.EVT_TOGGLEBUTTON, self.toggle_auto_focus)
        self._auto_focus = False

        toolbar.AddControl(wx.StaticText(toolbar, -1, "    "))
        toolbar.AddControl(toggle_auto_focus)

        toolbar.AddControl(wx.StaticText(toolbar,-1,"    "))
        toolbar.AddLabelTool(wx.ID_HELP, 'Help',
                wx.ArtProvider.GetBitmap(wx.ART_HELP,wx.ART_OTHER,(16,16)) )
        toolbar.AddLabelTool(wx.ID_SAVE, 'Save',
                wx.ArtProvider.GetBitmap(wx.ART_FILE_SAVE,wx.ART_OTHER,(16,16)) )
        toolbar.Realize()

        self.Bind(wx.EVT_TOOL, self.ShowControlsDialog, id=wx.ID_HELP)
        self.Bind(wx.EVT_TOOL, self.SaveDotGraph, id=wx.ID_SAVE)

        # Create dot graph widget
        self.widget = xdot.wxxdot.WxDotWindow(graph_view, -1)

        gv_vbox.Add(toolbar, 0, wx.EXPAND)
        gv_vbox.Add(self.widget, 1, wx.EXPAND)

        # Create tree view widget
        self.tree = wx.TreeCtrl(nb,-1,style=wx.TR_HAS_BUTTONS)
        self.tree.Bind(wx.EVT_TREE_SEL_CHANGED, self.OnTreeSelectionChanged)

        nb.AddPage(graph_view,"Graph View")
        nb.AddPage(self.tree,"Tree View")


        # Create userdata widget
        borders = wx.LEFT | wx.RIGHT | wx.TOP
        border = 4
        self.ud_win = wx.ScrolledWindow(self.content_splitter, -1)
        self.ud_gs = wx.BoxSizer(wx.VERTICAL)

        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Path:"),0, borders, border)

        self.path_input = wx.ComboBox(self.ud_win,-1,style=wx.CB_DROPDOWN)
        self.path_input.Bind(wx.EVT_COMBOBOX,self.selection_changed)
        self.ud_gs.Add(self.path_input,0,wx.EXPAND | borders, border)


        self.ud_gs.Add(wx.StaticText(self.ud_win,-1,"Userdata:"),0, borders, border)

        self.ud_txt = wx.TextCtrl(self.ud_win,-1,style=wx.TE_MULTILINE | wx.TE_READONLY)
        self.ud_gs.Add(self.ud_txt,1,wx.EXPAND | borders, border)
        
        # Add initial state button
        # self.is_button = wx.Button(self.ud_win,-1,"Set as Initial State")
        # self.is_button.Bind(wx.EVT_BUTTON, self.on_set_initial_state)
        # self.is_button.Disable()
        # self.ud_gs.Add(self.is_button,0,wx.EXPAND | wx.BOTTOM | borders, border)

        # Add trigger transition button
        self.tt_button = wx.Button(self.ud_win, -1, "Trigger Transition")
        self.tt_button.Bind(wx.EVT_BUTTON, self.on_trigger_transition)
        self.tt_button.Disable()
        self.ud_gs.Add(self.tt_button, 0, wx.EXPAND | wx.BOTTOM | borders, border)


        self.event_combo = wx.ComboBox(self.ud_win, -1, style=wx.CB_DROPDOWN)
        self.ud_gs.Add(self.event_combo, 0, wx.EXPAND | wx.BOTTOM | borders, border)

        # Add trigger event button
        self.event_button = wx.Button(self.ud_win, -1, "Trigger Event")
        self.event_button.Bind(wx.EVT_BUTTON, self.on_trigger_event)
        self.ud_gs.Add(self.event_button, 0, wx.EXPAND | wx.BOTTOM | borders, border)

        self.ud_win.SetSizer(self.ud_gs)


        # Set content splitter
        self.content_splitter.SplitVertically(viewer, self.ud_win, 512)

        # Add statusbar
        self.statusbar = wx.StatusBar(self,-1)

        # Add elements to sizer
        vbox.Add(self.content_splitter, 1, wx.EXPAND | wx.ALL)
        vbox.Add(self.statusbar, 0, wx.EXPAND)

        self.SetSizer(vbox)
        self.Center()

        # smach introspection client
        self._client = hsm.introspection.IntrospectionClient()
        self._containers= {}
        self._selected_paths = []

        # Message subscribers
        self._structure_subs = {}
        self._status_subs = {}

        self.Bind(wx.EVT_IDLE,self.OnIdle)
        self.Bind(wx.EVT_CLOSE,self.OnQuit)

        # Register mouse event callback
        self.widget.register_select_callback(self.select_cb)
        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True

        # Start a thread in the background to update the server list
        self._keep_running = True
        self._server_list_thread = threading.Thread(target=self._update_server_list)
        self._server_list_thread.start()

        # Wait till we have received the first structure message with a top container
        with self._update_cond:
            while not self._top_containers:
                if not self._update_cond.wait(5.0):
                    print('Waiting for a structure message containing a root node...')

        self._update_graph_thread = threading.Thread(target=self._update_graph)
        self._update_graph_thread.start()
        self._update_tree_thread = threading.Thread(target=self._update_tree)
        self._update_tree_thread.start()

        self.widget.Refresh()

    def OnQuit(self,event):
        """Quit Event: kill threads and wait for join."""
        with self._update_cond:
            self._keep_running = False
            self._update_cond.notify_all()

        self._server_list_thread.join()
        self._update_graph_thread.join()
        self._update_tree_thread.join()
        
        event.Skip()

    def update_graph(self):
        """Notify all that the graph needs to be updated."""
        with self._update_cond:
            self._update_cond.notify_all()

    def on_set_initial_state(self, event):
        """Event: Change the initial state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)
        state = get_label(state_path)

        server_name = self._containers[parent_path]._server_name
        self._client.set_initial_state(server_name,parent_path,[state],timeout = rospy.Duration(60.0))

    def on_trigger_transition(self, event):
        """Event: Change the current state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)

        server_name = self._containers[parent_path]._server_name
        transition_pub = rospy.Publisher(server_name + hsm.introspection.TRANSITION_TOPIC,
                                         String, queue_size=1)
        transition_msg = String()
        transition_msg.data = state_path
        transition_pub.publish(transition_msg)

    def on_trigger_event(self, event):
        """Event: Dispatch an event in the hsm."""
        server_name = self._containers[self._path]._server_name
        event_pub = rospy.Publisher(server_name + hsm.introspection.EVENT_TOPIC,
                                    String, queue_size=1)
        event_msg = String(self.event_combo.GetValue())
        event_pub.publish(event_msg)


    def set_path(self, event):
        """Event: Change the viewable path and update the graph."""
        self._path = self.path_combo.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_path(self, path):
        self._path = path
        self._needs_zoom = True
        self.path_combo.SetValue(path)
        self.update_graph()

    def set_depth(self, event):
        """Event: Change the maximum depth and update the graph."""
        self._max_depth = self.depth_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def _set_max_depth(self, max_depth):
        self._max_depth = max_depth
        self.depth_spinner.SetValue(max_depth)
        self._needs_zoom = True
        self.update_graph()

    def set_label_width(self, event):
        """Event: Change the label wrapper width and update the graph."""
        self._label_wrapper.width = self.width_spinner.GetValue()
        self._needs_zoom = True
        self.update_graph()

    def toggle_all_transitions(self, event):
        """Event: Change whether automatic transitions are hidden and update the graph."""
        self._show_all_transitions = not self._show_all_transitions
        self._structure_changed = True
        self.update_graph()

    def toggle_auto_focus(self, event):
        """Event: Enable/Disable automatically focusing"""
        self._auto_focus = not self._auto_focus
        self._needs_zoom = self._auto_focus
        self._structure_changed = True
        if not self._auto_focus:
            self._set_path('/')
            self._set_max_depth(-1)
        self.update_graph()

    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""

        # Only set string status
        if not type(item.url) is str:
            return

        self.statusbar.SetStatusText(item.url)
        if event.LeftDClick():
            self.on_trigger_transition(event)

        # Left button-up
        if event.LeftDown():
            # Update the selection dropdown
            self.path_input.SetValue(item.url)
            wx.PostEvent(
                    self.path_input.GetEventHandler(),
                    wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED,self.path_input.GetId()))
            self.update_graph()

    def selection_changed(self, event):
        """Event: Selection dropdown changed."""
        path_input_str = self.path_input.GetValue()
        # Store this item's url as the selected path
        self._selected_paths = [path_input_str]

        # Check the path is non-zero length
        if len(path_input_str) > 0:
            # Split the path (state:outcome), and get the state path
            path = path_input_str.split(':')[0]

            # Get the container corresponding to this path, since userdata is
            # stored in the containers
            if path not in self._containers:
                parent_path = get_parent_path(path)
            else:
                parent_path = path

            if parent_path in self._containers:
                # Enable the initial state button for the selection
                #self.is_button.Enable()
                self.tt_button.Enable()

                # Get the container
                container = self._containers[parent_path]

                # Store the scroll position and selection
                pos = self.ud_txt.HitTestPos(wx.Point(0,0))
                sel = self.ud_txt.GetSelection()

                # TODO remove ud_txt (we won't have it)
                # Restore the scroll position and selection
                self.ud_txt.ShowPosition(pos[1])
                if sel != (0,0):
                    self.ud_txt.SetSelection(sel[0],sel[1])
            else:
                # Disable the initial state button for this selection
                #self.is_button.Disable()
                self.tt_button.Disable()
        self.update_graph()

    def _init_structure(self, msg, server_name):
        """Initialize the structure of the server based on the given message."""
        # Just return if we're shutting down
        if not self._keep_running:
            return

        rospy.logdebug("STRUCTURE MSG WITH PREFIX: "+msg.prefix)

        with self._update_cond:
            self._build_container_tree(msg, server_name)

            # Update the graph
            self._structure_changed = True
            self._needs_zoom = True  # TODO: Make it so you can disable this
            self._update_cond.notify_all()

        # We want to wait until the first loop of ``self._update_graph`` and
        # ``self._update_tree`` is complete; only this way do we know the viewer
        # is set up, avoiding a race condition.
        while (self._structure_changed
               or not self._graph_update_complete
               or not self._tree_update_complete):
            with self._update_cond:
                self._update_cond.notify_all()
            rospy.sleep(0.2)

    def _build_container_tree(self, msg, server_name):
        """Build the structural tree of ``ContainerNode``s from the given
        structure ``msg``.
        """
        if msg.prefix:
            prefix = msg.prefix + '/'
        else:
            prefix = ''

        # Mapping from parent paths to list of child labels
        children_of = {}

        # We go through the states in reverse as we create the tree from the
        # leaves up to the root. We can do this because we know that our
        # ``msg_builder`` module creates the tree in pre-order.
        for state_msg in msg.states[::-1]:
            # Prefix has the '/' appended already.
            path = prefix + state_msg.path
            pathsplit = path.split('/')
            parent_path = '/'.join(pathsplit[0:-1])
            label = pathsplit[-1]

            rospy.logdebug("CONSTRUCTING: "+path)

            # Add to children of parent
            if parent_path not in children_of:
                children_of[parent_path] = []
            children_of[parent_path].append(label)

            # Create a new container
            container = ContainerNode(server_name,
                                      state_msg,
                                      prefix,
                                      children_of.get(path, []))
            self._containers[path] = container

            # Store this as a top container if it has no parent
            if not parent_path:
                self._top_containers[path] = container
                # Notify if we have our first top container.
                # TODO This is a pretty bad check; what if top containers are removed?
                if len(self._top_containers) == 1:
                    self._update_cond.notify_all()

            # Append paths to selector
            self.path_combo.Append(path)
            self.path_input.Append(path)

    # TODO Rework when we actually get updates
    def _structure_msg_update(self, msg, server_name):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""

        # Just return if we're shutting down
        if not self._keep_running:
            return

        # Get the node path
        path = msg.path
        pathsplit = path.split('/')
        parent_path = '/'.join(pathsplit[0:-1])

        rospy.logdebug("STRUCTURE MSG: "+path)
        rospy.logdebug("CONTAINERS: "+str(self._containers.keys()))

        # Initialize redraw flag
        needs_redraw = False

        with self._update_cond:
            if path in self._containers:
                rospy.logdebug("UPDATING: "+path)

                # Update the structure of this known container
                # We will never call this as the tree will be regenerated entirely
                needs_redraw = self._containers[path].update_structure(msg)
            else:
                rospy.logdebug("CONSTRUCTING: "+path)

                # Create a new container
                container = ContainerNode(server_name, msg)
                self._containers[path] = container

                # Store this as a top container if it has no parent
                if parent_path == '':
                    self._top_containers[path] = container
                    # Notify if we have our first top container.
                    # TODO This is a pretty bad check; what if top containers are removed?
                    if len(self._top_containers) == 1 and not self._containers:
                        self._update_cond.notify_all()

                # Append paths to selector
                self.path_combo.Append(path)
                self.path_input.Append(path)

                # We need to redraw thhe graph if this container's parent is already known
                if parent_path in self._containers:
                    needs_redraw = True

            # Update the graph if necessary
            if needs_redraw:
                self._structure_changed = True
                self._needs_zoom = True # TODO: Make it so you can disable this
                self._update_cond.notify_all()

    def _status_msg_update(self, msg):
        """Process status messages."""

        # Check if we're in the process of shutting down
        if not self._keep_running:
            return

        # Get the path to the updating conainer
        path = msg.path
        rospy.logdebug("STATUS MSG: "+path)

        # Check if this is a known container
        needs_update = False
        with self._update_cond:
            if path in self._containers:
                # Get the container and check if the status update requires regeneration
                container = self._containers[path]
                if not container.is_active:
                    prev_active_path = self._active_path
                    if self._active_path in self._containers:
                        self._containers[prev_active_path].is_active = False

                    # We update the parents even if the child is gone
                    self._update_parents(prev_active_path, False)

                    container.is_active = True
                    self._active_path = container._path

                    self._update_parents(path, True)
                    needs_update = True

            if needs_update:
                self._update_cond.notify_all()

    def _update_parents(self, path, set_active):
        """Go up the tree from the given path, setting all sequential parents
        to ``set_active`` and update their active children accordingly.
        """
        pathsplit = path.split('/')
        if set_active and path in self._containers:
            child = self._containers[path]
            active_children = [child._label]
        else:
            active_children = []
        del pathsplit[-1]

        while pathsplit:
            parent_path = '/'.join(pathsplit)

            # TODO Should we break in the else-case?
            if parent_path in self._containers:
                parent = self._containers[parent_path]
                parent.active_states = active_children

                # Update ``active_children``
                if set_active:
                    active_children = [parent._label]

            del pathsplit[-1]

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the SMACH plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated. 

          2: The status of the SMACH plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                # Wait for the update condition to be triggered
                self._update_cond.wait()

                # update the event combo box
                # TODO will we keep this in any way?
                # events = []
                # for c in self._containers.values():
                #     for o in c._internal_outcomes:
                #         events.append(o)
                # if not set(events) == set(self.event_combo.GetItems()):
                #     self.event_combo.Clear()
                #     for e in events:
                #         self.event_combo.Append(e)

                # Get the containers to update
                containers_to_update = {}
                if self._path in self._containers:
                    # Some non-root path
                    containers_to_update = {self._path:self._containers[self._path]}
                elif self._path == '/':
                    # Root path
                    containers_to_update = self._top_containers

                # Check if we need to re-generate the dotcode (if the structure changed)
                # TODO: needs_zoom is a misnomer
                if self._structure_changed or self._needs_zoom:
                    dotstr = "digraph {\n\t"
                    dotstr += ';'.join([
                        "compound=true",
                        "outputmode=nodesfirst",
                        "labeljust=l",
                        "nodesep=0.5",
                        "minlen=2",
                        "mclimit=5",
                        "clusterrank=local",
                        "ranksep=0.75",
                        # "remincross=true",
                        # "rank=sink",
                        "ordering=\"\"",
                        ])
                    dotstr += ";\n"

                    # Generate the rest of the graph
                    # TODO: Only re-generate dotcode for containers that have changed
                    for path,tc in containers_to_update.iteritems():
                        dotstr += tc.get_dotcode(
                                self._selected_paths,[],
                                0,self._max_depth,
                                self._containers,
                                self._show_all_transitions,
                                self._label_wrapper)

                    if len(containers_to_update) == 0:
                        dotstr += '"__empty__" [label="Path not available.", shape="plaintext"]'

                    dotstr += '\n}\n'
                    self.dotstr = dotstr
                    # Set the dotcode to the new dotcode, reset the flags
                    self.set_dotcode(dotstr,zoom=False)
                    self._structure_changed = False

                # Update the styles for the graph if there are any updates
                for path,tc in containers_to_update.iteritems():
                    tc.set_styles(
                            self._selected_paths,
                            0,self._max_depth,
                            self.widget.items_by_url,
                            self.widget.subgraph_shapes,
                            self._containers)

                # Redraw
                self.widget.Refresh()
                self._graph_update_complete = True

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self.widget.set_dotcode(dotcode, None):
            self.SetTitle('Smach Viewer')
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self.widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True
            wx.PostEvent(self.GetEventHandler(), wx.IdleEvent())

    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                modified = False
                for path,tc in self._top_containers.iteritems():
                    modified |= self.add_to_tree(path, None)
                    self.update_tree_status(tc)
                #self.tree.ExpandAll() (caused spurious crashes)
                self._tree_update_complete = True

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        modified = False
        container = self._tree_nodes.get(path, None)
        if container is None:
            if parent is None:
                container = self.tree.AddRoot(get_label(path))
            else:
                container = self.tree.AppendItem(parent,get_label(path))
            self._tree_nodes[path] = container
            modified = True

        # Add children to tree
        sub = self._containers.get(path, None)
        children = sub._children if sub is not None else []
        added_children = False
        for label in children:
            child_path = '/'.join([path,label])
            added_children |= self.add_to_tree(child_path, container)
            if added_children:
                self.tree.Expand(container)

        return modified | added_children

    def update_tree_status(self, tc):
        path = tc._path
        item = self._tree_nodes[path]
        self.tree.SetItemBold(item,
                              self._active_path.startswith(path))
        for child_label in tc._children:
            child_path = '/'.join([tc._path, child_label])
            child_item = self._tree_nodes[child_path]
            if child_path in self._containers:
                self.update_tree_status(self._containers[child_path])

    def OnIdle(self, event):
        """Event: On Idle, refresh the display if necessary, then un-set the flag."""
        if self._needs_refresh:
            self.Refresh()
            # Re-populate path combo
            self._needs_refresh = False

    def _update_server_list(self):
        """Update the list of known SMACH introspection servers."""
        while self._keep_running:
            # Update the server list
            server_names = self._client.get_servers()
            new_server_names = [sn for sn in server_names if sn not in self._status_subs]

            # Create subscribers for new servers
            for server_name in new_server_names:
                self._structure_subs[server_name] = rospy.Subscriber(
                        server_name+hsm.introspection.STRUCTURE_TOPIC,
                        msgs.HsmStructure,
                        callback = self._init_structure,
                        callback_args = server_name,
                        queue_size=50)

                self._status_subs[server_name] = rospy.Subscriber(
                        server_name+hsm.introspection.STATUS_TOPIC,
                        msgs.HsmStatus,
                        callback = self._status_msg_update,
                        queue_size=50)



            # This doesn't need to happen very often
            rospy.sleep(1.0)
            
            
            #self.server_combo.AppendItems([s for s in self._servers if s not in current_servers])

            # Grab the first server
            #current_value = self.server_combo.GetValue()
            #if current_value == '' and len(self._servers) > 0:
            #    self.server_combo.SetStringSelection(self._servers[0])
            #    self.set_server(self._servers[0])

    def ShowControlsDialog(self,event):
        dial = wx.MessageDialog(None,
                "Pan: Arrow Keys\nZoom: PageUp / PageDown\nZoom To Fit: F\nRefresh: R",
                'Keyboard Controls', wx.OK)
        dial.ShowModal()

    def SaveDotGraph(self,event):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = rospkg.get_ros_home()+'/dotfiles/'
        if not os.path.exists(directory):
                os.makedirs(directory)
        filename = directory+timestr+'.dot'
        print('Writing to file: %s' % filename)
        with open(filename, 'w') as f:
            f.write(self.dotstr)

    def OnExit(self, event):
        pass

    def OnTreeSelectionChanged(self, event):
        paths=[]
        item = event.GetItem()
        while item.IsOk():
            paths.append(self.tree.GetItemText(item))
            item = self.tree.GetItemParent(item)

        # Update the selection dropdown
        self.path_input.SetValue('/'.join(reversed(paths)))
        wx.PostEvent(
            self.path_input.GetEventHandler(),
            wx.CommandEvent(wx.wxEVT_COMMAND_COMBOBOX_SELECTED, self.path_input.GetId()))
        event.Skip()

    def set_filter(self, filter):
        self.widget.set_filter(filter)

def main():
    from argparse import ArgumentParser
    p = ArgumentParser()
    p.add_argument('-f', '--auto-focus',
                 action='store_true',
                 help="Enable 'AutoFocus to subgraph' as default",
                 dest='enable_auto_focus')
    args = p.parse_args()
    app = wx.App()

    frame = SmachViewerFrame()
    frame.set_filter('dot')

    frame.Show()

    if args.enable_auto_focus:
        frame.toggle_auto_focus(None)

    app.MainLoop()

if __name__ == '__main__':
    rospy.init_node('smach_viewer',anonymous=False, disable_signals=True,log_level=rospy.INFO)
    sys.argv = rospy.myargv()
    main()
