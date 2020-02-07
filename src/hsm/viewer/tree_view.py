from gtk_wrap import Gtk


class TreeView(Gtk.TreeView):
    """Tree view of all paths. Enables advanced interaction such as transition triggering."""

    TEXT_COLUMN = 0
    """The column containing the text shown."""
    WEIGHT_COLUMN = 1
    """The column containing the weight the text is rendered with."""

    def __init__(self, model, *args, **kwargs):
        """Initialize the tree view, enabling bold text."""
        Gtk.TreeView.__init__(self, model, *args, **kwargs)

        # Enable bold text
        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn(
            'Path',
            renderer,
            text=self.TEXT_COLUMN,
            weight=self.WEIGHT_COLUMN,
            weight_set=True
        )
        self.append_column(column)
