from __future__ import annotations
import cv2
import numpy as np
from .types import Dimension, ViewMeasurement, View
from .segment import Segmenter
from .subpixel import refine_contour

# which world axes each view's (width, height) map to
VIEW_AXES = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}


def measure_view(res, segmenter: Segmenter, view: View,
                 board_mask_px: tuple[float, float, float, float] | None = None) -> ViewMeasurement:
    """Measure the object's overall dimensions in a rectified metric image.

    The scene typically contains the board too; pass ``board_mask_px`` (the board's
    bounding box in rectified px) to exclude it so only the object is measured.
    """
    img = res.image.copy()
    if board_mask_px is not None:
        x0, y0, x1, y1 = (int(round(v)) for v in board_mask_px)
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = segmenter.segment(img)
    poly = refine_contour(gray, mask)

    # Views are rectified fronto-parallel and axis-aligned to the board, so the object's
    # HORIZONTAL extent is the view's width axis and the VERTICAL extent is its height axis.
    # (Using max/min instead would mislabel views whose height exceeds their width.)
    xs, ys = poly[:, 0], poly[:, 1]
    x0, x1 = float(xs.min()), float(xs.max())
    y0, y1 = float(ys.min()), float(ys.max())
    width_mm = (x1 - x0) * res.mm_per_px
    height_mm = (y1 - y0) * res.mm_per_px
    # combine 0.5 px edge-localisation with the fractional scale uncertainty
    sig = lambda v: float(np.hypot(0.5 * res.mm_per_px, v * res.sigma_scale))
    ax_w, ax_h = VIEW_AXES[view]
    dims = [
        Dimension("width", "linear", (x0, y1), (x1, y1),
                  round(width_mm, 2), round(sig(width_mm), 2), ax_w),
        Dimension("height", "linear", (x0, y0), (x0, y1),
                  round(height_mm, 2), round(sig(height_mm), 2), ax_h),
    ]
    warnings = [] if res.tilt_deg < 2.5 else [f"camera tilt {res.tilt_deg:.1f} deg > 2.5"]
    return ViewMeasurement(view, dims, poly, res.mm_per_px, warnings)
