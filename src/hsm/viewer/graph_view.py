from . import *
from . xdot import xdot


class GraphView(object):
    def __init__(self, builder, model):
        self.model = model
        toolbar = builder.get_object('graph_toolbar')
        box = toolbar.get_parent()
        self.dot_widget = xdot.DotWidget()
        box.pack_start(self.dot_widget, expand=True, fill=True, padding=0)

        self.filter_combo = builder.get_object('filter_combo')
        self.zoom_to_fit_btn = builder.get_object('zoom_to_fit')
        self.zoom_to_fit_btn.connect('toggled', self.zoom_to_fit)

    def zoom_to_fit(self, *args):
        if self.zoom_to_fit_btn.get_active():
            self.dot_widget.zoom_to_fit()
