import numpy as np
import yaml
from dimgram.types import Dimension, ViewMeasurement
from dimgram.bundle import write_bundle


def _vm(view):
    return ViewMeasurement(view, [Dimension("width", "linear", (0, 0), (1, 0), 100.0, 0.5, "X")],
                           np.zeros((3, 2)), 0.25, [])


def test_write_bundle_layout(tmp_path):
    views = {v: _vm(v) for v in ("front", "top", "side")}
    rectified = {v: np.zeros((10, 10, 3), np.uint8) for v in views}
    write_bundle(tmp_path / "bundle", views, rectified, mm_per_px=0.25,
                 board_yaml={"square_mm": 24.0}, camera_yaml={"model": "none"})
    b = tmp_path / "bundle"
    assert (b / "front.json").exists()
    assert (b / "rectified" / "top.png").exists()
    assert yaml.safe_load((b / "scale.yaml").read_text())["mm_per_px"] == 0.25
    assert (b / "camera.yaml").exists()
