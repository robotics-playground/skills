"""DXF (one layer per part, true SPLINE curves split at real corners) and simple SVG output."""
from __future__ import annotations
import numpy as np
import ezdxf
from shapely import affinity


def to_mm(g, axis_x_px, sole_y_px, mm_per_px, base_w_mm, base_h_mm):
    """Image px (y down) -> mm (origin base bottom-left, y up), centring the axis on the base."""
    g = affinity.translate(g, -axis_x_px, -sole_y_px); g = affinity.scale(g, mm_per_px, -mm_per_px, origin=(0, 0))
    return affinity.translate(g, base_w_mm / 2, base_h_mm)


def spline_segments(ring: np.ndarray, step=0.35, corner_deg=35.0):
    """Split a closed ring at corners; straight runs -> 2-point lines, the rest -> fit-point lists."""
    ring = ring[:-1] if np.allclose(ring[0], ring[-1]) else ring
    rr = np.vstack([ring, ring[:1]]); d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(rr, axis=0), axis=1))]
    t = np.arange(0, d[-1], 0.05); P = np.c_[np.interp(t, d, rr[:, 0]), np.interp(t, d, rr[:, 1])]; n = len(P); k = 6
    v1 = P - np.roll(P, k, 0); v2 = np.roll(P, -k, 0) - P
    ang = np.abs(np.degrees(np.arctan2(v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0], (v1 * v2).sum(1))))
    cs = []
    for i in np.argsort(-ang):
        if ang[i] < corner_deg: break
        if all(min(abs(i - j), n - abs(i - j)) > 3 * k for j in cs): cs.append(i)
    cs = sorted(cs); stride = max(1, int(step / 0.05))
    if not cs:
        pts = P[::stride]; return [np.vstack([pts, pts[:1]])]
    segs = []
    for a, b in zip(cs, cs[1:] + [cs[0] + n]):
        seg = P[np.r_[np.arange(a, b, stride), b] % n]; e = seg[-1] - seg[0]
        dev = np.abs(e[0] * (seg[1:-1] - seg[0])[:, 1] - e[1] * (seg[1:-1] - seg[0])[:, 0]).max() / max(np.linalg.norm(e), 1e-9) if len(seg) > 2 else 0
        segs.append(seg if dev > 0.02 else seg[[0, -1]])
    return segs


def write_dxf(parts: dict, path: str, aci: dict | None = None, splines: bool = True):
    doc = ezdxf.new(units=4); msp = doc.modelspace()
    for k, g in parts.items():
        doc.layers.add(k, color=(aci or {}).get(k, 7))
        for p in getattr(g, 'geoms', [g]):
            if p.is_empty: continue
            for r in [p.exterior, *p.interiors]:
                arr = np.array(r.coords)
                if not splines:
                    msp.add_lwpolyline(arr.round(4).tolist(), close=True, dxfattribs={'layer': k}); continue
                for seg in spline_segments(arr):
                    if len(seg) == 2: msp.add_line(tuple(seg[0]), tuple(seg[1]), dxfattribs={'layer': k})
                    else: msp.add_spline(fit_points=[(x, y, 0) for x, y in seg], dxfattribs={'layer': k})
    doc.saveas(path)


def svg_path(g, flip_y=True):
    out = ''
    for p in getattr(g, 'geoms', [g]):
        if p.is_empty: continue
        for r in [p.exterior, *p.interiors]:
            out += 'M' + ' L'.join(f'{x:.3f},{-y if flip_y else y:.3f}' for x, y in r.coords) + 'Z'
    return out
