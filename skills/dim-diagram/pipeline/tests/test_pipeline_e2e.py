"""End-to-end: 3 synthetic views of a known box -> reconciled X/Y/Z.

Guards two bugs the per-module tests missed:
  * axis mislabelling when a view's height exceeds its width (side view here), and
  * cross-view consistency (the box must reconcile without disagreement warnings).
"""
import cv2
import numpy as np
from tests.conftest import SQUARES_X, SQUARES_Y, SQUARE_MM
from dimgram.reference import detect_charuco
from dimgram.rectify import rectify_to_metric
from dimgram.segment import ClassicalSegmenter
from dimgram.measure import measure_view
from dimgram.reconcile import reconcile

# Known box: X(width)=120, Y(depth)=80, Z(height)=90 mm
VIEWS = {"front": (120, 90), "top": (120, 80), "side": (80, 90)}


def _scene(board, w_mm, h_mm, mm_per_px=0.25):
    px = lambda mm: int(round(mm / mm_per_px))
    bw, bh = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    m = px(20)
    img = np.full((m * 2 + px(bh), m * 3 + px(bw) + px(w_mm)), 255, np.uint8)
    img[m:m + px(bh), m:m + px(bw)] = board.generateImage((px(bw), px(bh)))
    cv2.rectangle(img, (m * 2 + px(bw), m), (m * 2 + px(bw) + px(w_mm), m + px(h_mm)), 0, -1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def _mask_box(det, res, pad=120):
    bc = cv2.perspectiveTransform(
        det.charuco_corners.reshape(-1, 1, 2).astype(np.float64), res.homography).reshape(-1, 2)
    return (bc[:, 0].min() - pad, bc[:, 1].min() - pad,
            bc[:, 0].max() + pad, bc[:, 1].max() + pad)


def test_three_views_reconcile_to_known_box(board):
    views = {}
    for name, (w, h) in VIEWS.items():
        img = _scene(board, w, h)
        det = detect_charuco(img, board)
        res = rectify_to_metric(img, det, target_px_per_mm=4.0)
        views[name] = measure_view(res, ClassicalSegmenter(), name,
                                   board_mask_px=_mask_box(det, res))
    out = reconcile(views)
    assert abs(out["axes_mm"]["X"] - 120) / 120 < 0.02
    assert abs(out["axes_mm"]["Y"] - 80) / 80 < 0.02
    assert abs(out["axes_mm"]["Z"] - 90) / 90 < 0.02
    assert out["warnings"] == []          # shared dims must agree across views
