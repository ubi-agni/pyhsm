from . import *
from . xdot import xdot
from textwrap import TextWrapper
from state_node import RootStateNode, DummyStateNode


def format_attrs(join=', ', **kwargs):
    return join.join(['{key}="{value}"'.format(key=k, value=v) for k,v in kwargs.iteritems()])


def hex2t(hex):
    """Convert a hexadecimal color string into a 4-tuple."""
    return [int(hex[i:i + 2], 16) / 255.0 for i in range(1, len(hex), 2)]


class GraphView(object):
    COLOR_ACTIVE = hex2t('#5C7600FF')
    COLOR_INACTIVE = hex2t('#808080FF')

    FILL_ACTIVE = hex2t('#C0F700FF')
    FILL_INACTIVE =hex2t('#FFFFFFFF')

    def __init__(self, builder, model):
        self.model = model
        self._dotstr = ''
        self._label_wrapper = TextWrapper(break_long_words=True)

        toolbar = builder.get_object('graph_toolbar')
        box = toolbar.get_parent()
        self.dot_widget = xdot.DotWidget()
        box.pack_start(self.dot_widget, expand=True, fill=True, padding=0)

        # HACK Unfortunately, builder.connect_signals() can only connect to a single class
        # So, need to connect manually here
        self.filter_combo = builder.get_object('filter_combo')
        self.zoom_to_fit_btn = builder.get_object('zoom_to_fit')
        self.depth_spinner = builder.get_object('depth_spinner')
        self.label_width_spinner = builder.get_object('label_width_spinner')
        self.save_dlg = builder.get_object('save_dlg')
        builder.get_object('save_btn').connect('clicked', self.save)

        self.zoom_to_fit_btn.connect('toggled', self.zoom_to_fit)
        self.filter_combo.connect('changed', self.update)
        self.depth_spinner.connect('value-changed', self.update)
        self.label_width_spinner.connect('value-changed', self.update)

    def zoom_to_fit(self, *args):
        if self.zoom_to_fit_btn.get_active():
            self.dot_widget.zoom_to_fit()

    def save(self, *args):
        dlg = self.save_dlg
        if not dlg.get_filename():
            dlg.set_current_name('hsm.dot')
        if dlg.run() == Gtk.ResponseType.ACCEPT:
            with open(dlg.get_filename(), 'w') as file:
                file.write(self._dotstr)
        dlg.hide()

    @staticmethod
    def id(state):
        root = state.root
        prefix = root and root.server_name + ':' or ''
        return prefix + state.path

    def dotcode(self, model):
        max_depth = self.depth_spinner.get_value_as_int()
        wrapper = self._label_wrapper
        wrapper.width = self.label_width_spinner.get_value_as_int()

        def hierarchy(item, depth=0):
            state = model.state(item)
            label = model.label(item)

            # recursively process children
            children = ''
            if max_depth == -1 or depth < max_depth:
                for child in model.children(item):
                    children += hierarchy(child, depth+1)

            # process current item
            attrs = dict()
            if isinstance(state, RootStateNode):
                attrs.update(style='filled')
                parent = model.iter_parent(item)
                parent = parent and model.state(parent)
                if isinstance(parent, DummyStateNode):
                    label = parent.path + '/' + label  # prepend dummy's path
            elif isinstance(state, DummyStateNode):
                return children  # skip dummy nodes
            else:
                attrs.update(style='filled,rounded')

            return '''
{indent}subgraph "cluster {id}" {{ {cluster_attrs}
{indent}\t"__S {id}" [{node_attrs}]
{children}
{indent}}}'''.format(indent='\t' * (depth + 1), id=self.id(state), label=label, children=children,
                     cluster_attrs=format_attrs(**attrs),
                     node_attrs=format_attrs(label='\\n'.join(wrapper.wrap(label))), url=state.path)

        # Generate state hierarchy H
        if model.iter_children() is None:
            H = '\t"__empty__" [{attrs}]'.format(attrs=format_attrs(label='Waiting for HSM', fontcolor='gray'))
        else:
            filter = self.filter_combo.get_active_iter()
            items = [filter] if filter else model.children(None)
            H = ''
            for item in items:
                H += hierarchy(item)
            H = H or '\t"__empty__" [{attrs}]'.format(attrs=format_attrs(label='Path not available', fontcolor='red'))

        global_opts = format_attrs(join='\n\t',
                                   compound='true',
                                   outputmode='nodesfirst',
                                   labeljust='l',
                                   nodesep=0.5,
                                   minlen=2,
                                   mclimit=5,
                                   clusterrank='local',
                                   ranksep=0.75,
                                   color='#000000FF',
                                   fillcolor='#FFFFFF00',
                                   ordering='')
        return '''digraph {{
\t{global_opts}
\tnode [shape=plaintext]

{hierarchy}
}}'''.format(hierarchy=H, global_opts=global_opts)

    def update(self, *args):
        self._dotstr = self.dotcode(self.model)
        self.dot_widget.set_dotcode(self._dotstr)
        self.update_styles(self.model)
        self.zoom_to_fit()

    def update_styles(self, model):
        """Update the coloring of the shapes in the generated graph."""
        max_depth = self.depth_spinner.get_value_as_int()

        def traverse(item, depth=0):
            # recursively process children
            if max_depth == -1 or depth < max_depth:
                for child in model.children(item):
                    traverse(child, depth+1)

            # process current item
            self.update_style(model, item)

        filter = self.filter_combo.get_active_iter()
        items = [filter] if filter else model.children(None)
        for item in items:
            traverse(item)

        self.dot_widget.queue_draw()

    def update_style(self, model, item):
        state = model.state(item)
        if isinstance(state, DummyStateNode):
            return  # skip dummy nodes

        id = self.id(state)
        active = model.get_value(item, model.WEIGHT) > Pango.Weight.NORMAL
        linewidth = 4 if active else 2
        color = self.COLOR_ACTIVE if active else self.COLOR_INACTIVE
        fillcolor = self.FILL_ACTIVE if active else self.FILL_INACTIVE

        for shape in self.dot_widget.graph.subgraph_shapes.get('cluster ' + id, []):
            pen = shape.pen
            pen.color = color[:len(pen.color)]
            pen.fillcolor = fillcolor[:len(pen.fillcolor)]
            pen.linewidth = linewidth
