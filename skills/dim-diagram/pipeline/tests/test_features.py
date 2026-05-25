import cv2
import numpy as np
from dimgram.rectify import RectifyResult
from dimgram.segment import ClassicalSegmenter
from dimgram.measure import measure_view, detect_holes


def _part_with_hole(mm_per_px=0.25):
    """Black 120x90 mm part on white with a 30 mm-diameter round hole near the centre."""
    px = lambda mm: int(round(mm / mm_per_px))
    img = np.full((px(90) + 40, px(120) + 40, 3), 255, np.uint8)
    cv2.rectangle(img, (20, 20), (20 + px(120), 20 + px(90)), (0, 0, 0), -1)
    cv2.circle(img, (20 + px(60), 20 + px(45)), px(30) // 2, (255, 255, 255), -1)
    return img


def _fake_res(img, mm_per_px=0.25):
    return RectifyResult(img, mm_per_px, np.eye(3), 0.0, 0.001)


def test_detect_hole_diameter_within_3pct():
    img = _part_with_hole()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    holes = detect_holes(gray, mm_per_px=0.25, sigma_scale=0.001)
    assert len(holes) == 1
    assert holes[0].kind == "diameter"
    assert abs(holes[0].value_mm - 30.0) / 30.0 < 0.03


def test_measure_view_includes_hole_and_overall():
    vm = measure_view(_fake_res(_part_with_hole()), ClassicalSegmenter(), "front")
    labels = {d.label for d in vm.dimensions}
    assert {"width", "height"} <= labels
    dia = [d for d in vm.dimensions if d.kind == "diameter"]
    assert len(dia) == 1 and abs(dia[0].value_mm - 30.0) / 30.0 < 0.03
    # overall still correct: 120 x 90
    dims = {d.label: d.value_mm for d in vm.dimensions if d.kind == "linear"}
    assert abs(dims["width"] - 120) / 120 < 0.02 and abs(dims["height"] - 90) / 90 < 0.02
