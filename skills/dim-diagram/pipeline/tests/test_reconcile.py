import numpy as np
from dimgram.types import Dimension, ViewMeasurement
from dimgram.reconcile import reconcile


def _vm(view, width, height):
    ax = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}[view]
    return ViewMeasurement(view,
        [Dimension("width", "linear", (0, 0), (1, 0), width, 0.5, ax[0]),
         Dimension("height", "linear", (0, 0), (0, 1), height, 0.5, ax[1])],
        np.zeros((3, 2)), 0.25, [])


def test_reconcile_averages_shared_axes_and_flags_disagreement():
    views = {"front": _vm("front", 100.0, 60.0),
             "top":   _vm("top",   102.0, 40.0),     # X (width) shared with front
             "side":  _vm("side",  40.0, 60.0)}       # Y shared w/ top, Z shared w/ front
    out = reconcile(views)
    assert abs(out["axes_mm"]["X"] - 101.0) < 0.01    # averaged 100 & 102
    assert abs(out["axes_mm"]["Z"] - 60.0) < 0.01
    assert abs(out["axes_mm"]["Y"] - 40.0) < 0.01
    assert any("X" in w for w in out["warnings"])      # 2% disagreement flagged
