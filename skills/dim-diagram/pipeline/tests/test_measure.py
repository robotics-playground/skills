import cv2
import numpy as np
from dimgram.reference import detect_charuco
from dimgram.rectify import rectify_to_metric
from dimgram.segment import ClassicalSegmenter
from dimgram.measure import measure_view


def _board_mask_box(det, res, pad=120):
    bc = cv2.perspectiveTransform(
        det.charuco_corners.reshape(-1, 1, 2).astype(np.float64),
        res.homography).reshape(-1, 2)
    return (bc[:, 0].min() - pad, bc[:, 1].min() - pad,
            bc[:, 0].max() + pad, bc[:, 1].max() + pad)


def test_measure_recovers_rectangle_within_2pct(synthetic_scene, board):
    img, _, rect_mm = synthetic_scene
    det = detect_charuco(img, board)
    res = rectify_to_metric(img, det, target_px_per_mm=4.0)
    vm = measure_view(res, ClassicalSegmenter(), view="front",
                      board_mask_px=_board_mask_box(det, res))
    dims = {d.label: d.value_mm for d in vm.dimensions}
    assert abs(dims["width"] - 100.0) / 100.0 < 0.02
    assert abs(dims["height"] - 60.0) / 60.0 < 0.02
    assert all(d.sigma_mm > 0 for d in vm.dimensions)
    assert vm.dimensions[0].axis in ("X", "Y", "Z")
