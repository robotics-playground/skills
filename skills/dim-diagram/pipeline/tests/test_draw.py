from pathlib import Path
import numpy as np
from dimgram.types import Dimension, ViewMeasurement
from dimgram.draw import render_sheet


def _vm(view):
    ax = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}[view]
    contour = np.array([[0, 0], [100, 0], [100, 60], [0, 60]], float)
    return ViewMeasurement(view,
        [Dimension("width", "linear", (0, 0), (100, 0), 100.0, 0.5, ax[0]),
         Dimension("height", "linear", (0, 0), (0, 60), 60.0, 0.5, ax[1])],
        contour, 0.25, [])


def test_render_sheet_writes_all_formats(tmp_path):
    views = {v: _vm(v) for v in ("front", "top", "side")}
    out = render_sheet(views, tmp_path / "drawing", title="Test Part",
                       formats=("svg", "png", "pdf", "dxf"))
    for ext in ("svg", "png", "pdf", "dxf"):
        assert Path(out[ext]).exists() and Path(out[ext]).stat().st_size > 0
