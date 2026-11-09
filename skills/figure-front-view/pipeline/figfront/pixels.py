"""Sub-pixel edge positions read straight from the artwork pixels.

Use for small drawn features at the resolution limit (1-4 px ankle steps, a 0.6 px notch between two fists):
tracing + smoothing rounds them, but the 50 % coverage crossing per row or column locates each edge to
~0.1 px. Measure both mirrored copies (left and right boot) and average them for a symmetric part.
"""
from __future__ import annotations
import numpy as np


def coverage(img_bgr, paint_fg, paint_bg):
    """0..1 share of paint_fg in each pixel, assuming a two-paint blend (anti-aliasing)."""
    fg = np.asarray(paint_fg, float); bg = np.asarray(paint_bg, float); v = fg - bg
    return np.clip(((np.asarray(img_bgr, float) - bg) @ v) / (v @ v), 0, 1)


def row_crossing(cov, row, x_from, x_to):
    """x (pixel-edge coordinates: pixel i covers [i, i+1]) where coverage crosses 0.5 in `row`, scanning
    from x_from towards x_to. NaN if there is no crossing."""
    st = 1 if x_to > x_from else -1
    for x in range(x_from, x_to, st):
        a0, a1 = cov[row, x], cov[row, x + st]
        if (a0 - .5) * (a1 - .5) <= 0 and a0 != a1:
            return x + .5 + st * (.5 - a0) / (a1 - a0)
    return float("nan")


def col_crossing(cov, col, y_from, y_to):
    """y where coverage crosses 0.5 in column `col`, scanning from y_from towards y_to."""
    return row_crossing(cov.T, col, y_from, y_to)
