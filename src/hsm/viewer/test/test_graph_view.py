from hsm.viewer.graph_view import GraphView, hex2c, c2hex


def test_color_conversion():
    for color in [GraphView.COLOR_ACTIVE, GraphView.COLOR_INACTIVE,
                  GraphView.FILL_ACTIVE, GraphView.FILL_INACTIVE, GraphView.COLOR_SELECTED]:
        assert color == hex2c(c2hex(color))