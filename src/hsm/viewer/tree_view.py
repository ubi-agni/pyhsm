from gtk_wrap import Gtk


class TreeView(Gtk.TreeView):
    """Tree view of all paths. Enables advanced interaction such as transition triggering."""

    def __init__(self, model, *args, **kwargs):
        """Initialize the tree view, enabling bold text."""
        Gtk.TreeView.__init__(self, model, *args, **kwargs)

        # Enable bold text
        renderer = Gtk.CellRendererText()
        renderer.set_property('weight_set', True)
        column = Gtk.TreeViewColumn(
            'Path',
            renderer,
        )
        column.set_cell_data_func(renderer, model.render_path)
        self.append_column(column)
