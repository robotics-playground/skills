"""Stacking order -> what is visible, and hidden-part completion."""
from __future__ import annotations
import numpy as np
from shapely.ops import unary_union
from .contours import disk_op


def visible_regions(parts: dict, order: list[str]) -> dict:
    """order is back -> front. Visible = part minus every part in front: gap-free, non-overlapping colour
    regions for a multi-colour print."""
    out = {}
    for i, k in enumerate(order):
        front = [parts[j] for j in order[i + 1:] if j in parts]
        out[k] = parts[k].difference(unary_union(front)).buffer(0) if front else parts[k]
    return out


def complete_hidden(vis: np.ndarray, prim: np.ndarray, front: np.ndarray, tuck_px: float, margin_px: float) -> np.ndarray:
    """Full part mask = visible pixels + (fitted primitive OR visible grown by `tuck_px`) where a part in
    front covers it. Tuck must be generous (3 px worked; 1.5 px left white gaps by the helmet)."""
    hidden = (prim | disk_op(vis, 'dil', tuck_px)) & disk_op(front, 'dil', margin_px)
    return vis | hidden


def weld_near_front(full: np.ndarray, vis: np.ndarray, front: np.ndarray, r_close: float, r_open: float, near_r: float) -> np.ndarray:
    """Fill seams / remove spurs ONLY next to front parts, so visible edges stay exactly as traced."""
    if not front.any(): return full
    near = disk_op(front, 'dil', near_r)
    f1 = full | (disk_op(full, 'close', r_close) & near)
    return (disk_op(f1, 'open', r_open) & near) | (f1 & ~near) | (vis & ~near)
