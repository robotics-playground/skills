import numpy as np
from dimgram.types import Dimension, ViewMeasurement, to_json, from_json


def test_dimension_roundtrip(tmp_path):
    vm = ViewMeasurement(
        view="front",
        dimensions=[Dimension(label="width", kind="linear",
                              p1=(0.0, 0.0), p2=(100.0, 0.0),
                              value_mm=100.0, sigma_mm=0.5, axis="X")],
        contour=np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 50.0]]),
        mm_per_px=0.5,
        warnings=["tilt 1.2 deg"],
    )
    path = tmp_path / "front.json"
    to_json(vm, path)
    loaded = from_json(path)
    assert loaded.view == "front"
    assert loaded.dimensions[0].value_mm == 100.0
    assert loaded.dimensions[0].axis == "X"
    assert loaded.mm_per_px == 0.5
    np.testing.assert_allclose(loaded.contour, vm.contour)
