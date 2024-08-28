from . import *
from .xdot import xdot
from six import iteritems
from textwrap import TextWrapper
from .state_node import RootStateNode, DummyStateNode
from ..introspection import HISTORY_TRANSITION_MAGIC_WORD


def format_attrs(join="; ", **kwargs):
    return join.join(['{key}="{value}"'.format(key=k, value=v) for k, v in iteritems(kwargs)])


def hex2c(hex):
    """Convert a hexadecimal color string into a color 4-tuple."""
    return [int(hex[i : i + 2], 16) / 255.0 for i in range(1, len(hex), 2)]


def c2hex(color):
    """Convert a hexadecimal color string into a 4-tuple."""
    return "#" + "".join(map(lambda c: "%0.2X" % int(c * 255), color))


class DotWidget(xdot.DotWidget):
    """Customized DotWidget overriding some virtual methods"""

    __gsignals__ = {"activated": (GObject.SIGNAL_RUN_LAST, None, (str, object))}

    def __init__(self, *args, **kwargs):
        xdot.DotWidget.__init__(self, *args, **kwargs)

    def on_area_button_press(self, area, event):
        if event.type == Gdk.EventType._2BUTTON_PRESS:
            if event.button == 1:
                url = self.get_url(event.x, event.y)
                if url is not None:
                    self.emit("activated", url.url.decode("utf-8"), event)
            return True  # mark event as processed
        return xdot.DotWidget.on_area_button_press(self, area, event)

    def on_click(self, element, event):
        if event.button == 1:
            url = self.get_url(event.x, event.y)
            if url is not None:
                self.emit("clicked", url.url.decode("utf-8"), event)
        return True  # mark event as processed

    def error_dialog(self, error):
        pass


class GraphView(object):
    COLOR_SELECTED = hex2c("#FB000DFF")
    COLOR_ACTIVE = hex2c("#5C7600FF")
    COLOR_INACTIVE = hex2c("#404040FF")

    FILL_ACTIVE = hex2c("#C0F700FF")
    FILL_ROOT = hex2c("#DDDDDDFF")
    FILL_INACTIVE = hex2c("#FFFFFFFF")

    def __init__(self, builder, model):
        self.model = model
        self._label_wrapper = TextWrapper(break_long_words=True)
        self._selected_id = None

        toolbar = builder.get_object("graph_toolbar")
        box = toolbar.get_parent()
        self.dot_widget = DotWidget()
        box.pack_start(self.dot_widget, expand=True, fill=True, padding=0)

        # HACK Unfortunately, builder.connect_signals() can only connect to a single class
        # So, need to connect manually here
        self.filter_combo = builder.get_object("filter_combo")
        self.zoom_to_fit_btn = builder.get_object("zoom_to_fit")
        self.show_edges_btn = builder.get_object("show_edges")
        self.depth_spinner = builder.get_object("depth_spinner")
        self.label_width_spinner = builder.get_object("label_width_spinner")
        self.save_dlg = builder.get_object("save_dlg")
        builder.get_object("save_btn").connect("clicked", self.save)

        self.zoom_to_fit_btn.connect("toggled", self.zoom_to_fit)
        self.show_edges_btn.connect("toggled", self.update)
        self.filter_combo.connect("changed", self.update)
        self.depth_spinner.connect("value-changed", self.update)
        self.label_width_spinner.connect("value-changed", self.update)

    def zoom_to_fit(self, *args):
        if self.zoom_to_fit_btn.get_active():
            self.dot_widget.zoom_to_fit()

    def save(self, *args):
        dlg = self.save_dlg
        if not dlg.get_filename():
            dlg.set_current_name("hsm.dot")
        if dlg.run() == Gtk.ResponseType.ACCEPT:
            with open(dlg.get_filename(), "w") as file:
                file.write(self.dotcode(self.model))
        dlg.hide()

    @staticmethod
    def id(state):
        root = state.root
        prefix = root and root.server_name + ":" or ""
        return prefix + state.path

    def set_selected(self, item):
        id = self.id(self.model.state(item))
        if id != self._selected_id:
            self._selected_id = id
            self.update_styles(self.model)

    def dotcode(self, model):
        """Create dot code for HSM. See https://www.graphviz.org/doc/info for dot documentation."""
        max_depth = self.depth_spinner.get_value_as_int()
        wrapper = self._label_wrapper
        wrapper.width = self.label_width_spinner.get_value_as_int()
        show_edges = self.show_edges_btn.get_active()

        def hierarchy(item, depth=0):
            """Create hierarchical graph structure. The hierarchy is determined by edges from parent to children."""
            # recursively process children
            children = ""
            if max_depth == -1 or depth < max_depth:
                for child in model.children(item):
                    children += hierarchy(child, depth + 1)

            # process current item
            state = model.state(item)
            if isinstance(state, DummyStateNode):
                return children  # skip dummy nodes

            label = model.label(item)
            id = self.id(state)
            color, fillcolor, linewidth = self.get_style(model, item, state)
            attrs = dict(
                style="filled", penwidth=linewidth, color=c2hex(color), fillcolor=c2hex(fillcolor)
            )
            if isinstance(state, RootStateNode):
                parent = model.iter_parent(item)
                parent = parent and model.state(parent)
                if isinstance(parent, DummyStateNode):
                    label = parent.path + "/" + label  # prepend dummy's path
            else:
                attrs.update(style=attrs["style"] + ",rounded")

            return """
{indent}subgraph "cluster {id}" {{ {cluster_attrs}
{indent}\t"{id}" [{node_attrs}]
{indent}\t"{id}" -> {{{children}}}
{indent}}}""".format(
                indent="\t" * (depth + 1),
                id=id,
                label=label,
                children=children,
                cluster_attrs=format_attrs(**attrs),
                node_attrs=format_attrs(label="\\n".join(wrapper.wrap(label)), URL=id),
            )

        def transitions(item, depth=0):
            """Create edges for all transitions"""
            # process current item
            state = model.state(item)
            # prefix for target states is id(state.root) w/o root's label
            root = state.root and model.root_from_root_state(state.root)
            prefix = root and self.id(state.root)[: -len(self.model.label(root))] or ""

            result = ""
            if not isinstance(state, DummyStateNode):
                source = self.id(state)
                for idx, t in enumerate(state.transitions):
                    edge_atts = dict(constraint="false", style="", URL="_E_{} {}".format(idx, source))
                    target = prefix + t.target
                    if target.endswith(HISTORY_TRANSITION_MAGIC_WORD):
                        target = target[: -len(HISTORY_TRANSITION_MAGIC_WORD)]  # truncate magic word
                        edge_atts.update(color="blue")
                    # Disabled to suppress warnings "head is inside tail cluster"
                    # edge_atts.update(ltail="cluster " + source)
                    edge_atts.update(lhead="cluster " + target)
                    result += '\n\t"{src}" -> "{tgt}" [{edge_atts}]'.format(
                        src=source, tgt=target, edge_atts=format_attrs(**edge_atts)
                    )

            # recursively process children
            if max_depth == -1 or depth < max_depth:
                for child in model.children(item):
                    result += transitions(child, depth + 1)

            return result

        # Generate content
        if model.iter_children() is None:
            content = '\t"__empty__" [{attrs}]'.format(
                attrs=format_attrs(label="Waiting for HSM", fontcolor="gray")
            )
        else:
            filter = self.filter_combo.get_active_iter()
            items = [filter] if filter else model.children(None)

            # state hierarchy
            content = ""
            for item in items:
                content += hierarchy(item)
                if show_edges:
                    content += transitions(item)
            content = content or '\t"__empty__" [{attrs}]'.format(
                attrs=format_attrs(label="Path not available", fontcolor="red")
            )

        global_opts = format_attrs(
            join="\n\t",
            clusterrank="local",  # consider cluster subgraphs (default)
            compound="true",  # end arrows at cluster borders (together with lhead + ltail)
            outputorder="nodesfirst",  # draw edges on top of nodes
            ranksep=0.1,  # minimum distance between different levels
            nodesep=0,  # distanc between nodes at same rank
            margin=5,  # distance between clusters (in points)
        )
        return """digraph {{
\t{global_opts}
\tnode [shape=plaintext, margin=0, height=0.1]
\tedge [style=invis]

{content}
}}""".format(
            content=content, global_opts=global_opts
        )

    def update(self, *args):
        self.dot_widget.set_dotcode(self.dotcode(self.model))
        self.update_styles(self.model)
        self.zoom_to_fit()

    def update_styles(self, model):
        """Update the coloring of the shapes in the generated graph."""
        max_depth = self.depth_spinner.get_value_as_int()

        def traverse(item, depth=0):
            # recursively process children
            if max_depth == -1 or depth < max_depth:
                for child in model.children(item):
                    traverse(child, depth + 1)

            # process current item
            self.update_style(model, item)

        filter = self.filter_combo.get_active_iter()
        items = [filter] if filter else model.children(None)
        for item in items:
            traverse(item)

        self.dot_widget.queue_draw()

    @staticmethod
    def get_style(model, item, state=None):
        state = state and model.state(item)
        parent = model.iter_parent(item)
        parent = parent and model.state(parent)

        is_active = model.get_value(item, model.WEIGHT) > Pango.Weight.NORMAL
        is_root = isinstance(state, RootStateNode)
        try:
            is_initial = state.path.endswith("/" + parent.initial)
        except AttributeError:  # if parent is None or a dummy node, parent.initial will fail
            is_initial = False

        color = GraphView.COLOR_ACTIVE if is_active else GraphView.COLOR_INACTIVE
        fillcolor = (
            GraphView.FILL_ACTIVE
            if is_active
            else GraphView.FILL_ROOT if is_root else GraphView.FILL_INACTIVE
        )
        linewidth = 2 if is_active or is_initial else 1
        return color, fillcolor, linewidth

    def update_style(self, model, item):
        state = model.state(item)
        if isinstance(state, DummyStateNode):
            return  # skip dummy nodes

        color, fillcolor, linewidth = self.get_style(model, item, state)
        id = self.id(state)
        if id == self._selected_id:
            color = self.COLOR_SELECTED

        for shape in self.dot_widget.graph.subgraph_shapes.get(b"cluster " + id.encode("utf-8"), []):
            pen = shape.pen
            pen.color = color[: len(pen.color)]
            pen.fillcolor = fillcolor[: len(pen.fillcolor)]
            pen.linewidth = linewidth
