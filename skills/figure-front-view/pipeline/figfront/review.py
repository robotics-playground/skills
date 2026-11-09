"""One consolidated review sheet with numbered markers, so the user approves decisions in ONE pass
instead of pointing at problems one screenshot at a time."""
from __future__ import annotations
import numpy as np
import cv2


def _plain(v):
    """JSON-safe copy of edge parameters (numpy, tuples, nested lists)."""
    if isinstance(v, dict): return {k: _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [_plain(x) for x in v]
    if isinstance(v, np.ndarray): return np.round(v.astype(float), 3).tolist()
    if isinstance(v, (np.floating, float)): return round(float(v), 3)
    if isinstance(v, np.integer): return int(v)
    return v


def review_sheet(bg_bgr, parts_px: dict, edges_px: dict, origin=(0, 0), scale: int = 5, out_png: str = 'review_sheet.png'):
    """bg_bgr: artwork crop; parts_px: {part: polygon in image px}; edges_px: {part: [(where, Edge), ...]}
    from regularize.regularize_polygon. Draws outlines, numbers every edge (colour = kind) and returns the
    legend rows [{id, part, kind, params, dev_px}] to show next to the image."""
    x0, y0 = origin
    img = cv2.resize(bg_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    img = (img * 0.55 + 110).astype(np.uint8)
    col = {'line': (0, 140, 0), 'arc': (200, 0, 200), 'pieces': (200, 120, 0), 'spline': (0, 0, 220)}
    rows = []; n = 0
    for part, es in edges_px.items():
        for where, e in es:
            n += 1
            P = ((e.pts - [x0, y0]) * scale).round().astype(np.int32)
            cv2.polylines(img, [P], False, col.get(e.kind, (0, 0, 0)), 2, cv2.LINE_AA)
            mx, my = P[len(P) // 2]
            cv2.circle(img, (int(mx), int(my)), 9, (255, 255, 255), -1); cv2.circle(img, (int(mx), int(my)), 9, (0, 0, 0), 1)
            cv2.putText(img, str(n), (int(mx) - 6, int(my) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)
            rows.append(dict(id=n, part=part, where=where, kind=e.kind, dev_px=round(float(e.dev), 3), params=_plain(e.params)))
            cv2.circle(img, tuple(P[0]), 4, (0, 0, 0), -1)        # corner dots at run ends
    cv2.imwrite(out_png, img)
    return rows
