from __future__ import annotations
import cv2
import numpy as np
from .types import Dimension, ViewMeasurement, View
from .segment import Segmenter
from .subpixel import refine_contour

# which world axes each view's (width, height) map to
VIEW_AXES = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}


def detect_holes(gray: np.ndarray, mm_per_px: float, sigma_scale: float,
                 min_dia_mm: float = 4.0) -> list[Dimension]:
    """Find round holes (internal contours) and return their diameters.

    Holes are interior contours of the object; we keep only sufficiently large, round
    ones and report an area-equivalent diameter (robust for circles)."""
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    contours, hierarchy = cv2.findContours(th, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    dims: list[Dimension] = []
    if hierarchy is None:
        return dims
    min_area = np.pi * (min_dia_mm / mm_per_px / 2.0) ** 2
    for c, h in zip(contours, hierarchy[0]):
        if h[3] == -1:                      # external contour (the object) -> skip
            continue
        area = cv2.contourArea(c)
        peri = cv2.arcLength(c, True)
        if area < min_area or peri == 0:
            continue
        if 4 * np.pi * area / (peri * peri) < 0.7:   # circularity gate -> only round holes
            continue
        (cx, cy), _ = cv2.minEnclosingCircle(c)
        d_mm = 2.0 * np.sqrt(area / np.pi) * mm_per_px
        sig = float(np.hypot(0.5 * mm_per_px, d_mm * sigma_scale))
        dims.append(Dimension("hole_dia", "diameter", (float(cx), float(cy)),
                              (float(cx), float(cy)), round(d_mm, 2), round(sig, 2), None))
    return dims


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
    dims += detect_holes(gray, res.mm_per_px, res.sigma_scale)
    warnings = [] if res.tilt_deg < 2.5 else [f"camera tilt {res.tilt_deg:.1f} deg > 2.5"]
    return ViewMeasurement(view, dims, poly, res.mm_per_px, warnings)
