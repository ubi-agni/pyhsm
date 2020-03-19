from gtk_wrap import Gtk


class TreeView(Gtk.TreeView):
    """Tree view of all paths. Enables advanced interaction such as transition triggering."""

    def __init__(self, model, *args, **kwargs):
        Gtk.TreeView.__init__(self, model, *args, **kwargs)

        # Configure rendering: display label as text, use weight
        renderer = Gtk.CellRendererText()
        column = Gtk.TreeViewColumn('Path', renderer, text=model.LABEL, sensitive=model.ENABLED, weight=model.WEIGHT)
        column.set_sort_column_id(model.LABEL)  # sort by label
        self.append_column(column)

        # Disable selection of disabled rows
        selection = self.get_selection()
        def select_function(selection, model, path, *args):
            return model[path][model.ENABLED]
        selection.set_select_function(select_function)
