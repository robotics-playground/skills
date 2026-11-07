"""Masks -> clean sub-pixel outlines that keep every real drawn feature.

Rules learned the hard way:
  * Use ROUND (disk) structuring elements. scipy's default `iterations=` morphology is a diamond and
    cuts flat facets into smooth curves; corner detection then "sharpens" those facets.
  * Smoothing is BOUNDED by the measured tolerance (see calibrate.metrics): the curve may never move
    more than `max_shift_px` from the trace, so features bigger than ~1/4 effective pixel survive.
  * A point is a REAL corner only if both sides are straight AND keep their direction. Round tips
    (thumbs) also turn sharply over a short distance, but their sides keep turning; splitting a
    contour at them makes cusps/spikes.
  * The image's softness rounds real corners over ~1 effective pixel. Rebuild them by intersecting the
    two straight sides (vertex moves at most ~1 effective pixel).
"""
from __future__ import annotations
import numpy as np
import cv2
from scipy import ndimage as ndi
from scipy.interpolate import splprep, splev
from skimage import measure
from shapely.geometry import Polygon, LineString


def disk_op(m: np.ndarray, op: str, r: float) -> np.ndarray:
    """Round morphology: op in {'dil','ero','open','close'}, radius r in grid px."""
    r = max(1, int(round(r))); k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)); u = m.astype(np.uint8)
    f = {'dil': lambda: cv2.dilate(u, k), 'ero': lambda: cv2.erode(u, k),
         'open': lambda: cv2.morphologyEx(u, cv2.MORPH_OPEN, k), 'close': lambda: cv2.morphologyEx(u, cv2.MORPH_CLOSE, k)}[op]
    return f().astype(bool)


def mask_rings(mask: np.ndarray, scale: int = 1, origin=(0.0, 0.0), sigma: float = 1.4, min_area: float = 2.0):
    """Sub-pixel closed contours (in ORIGINAL image px) of a boolean mask drawn on a `scale`x grid."""
    f = ndi.gaussian_filter(mask.astype(float), sigma)
    rings = [np.c_[c[:, 1] - 2, c[:, 0] - 2] / scale + np.asarray(origin) for c in measure.find_contours(np.pad(f, 2), 0.5)]
    return [r for r in rings if len(r) > 6 and Polygon(r).area > min_area]


def resample(p: np.ndarray, step: float) -> np.ndarray:
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]; t = np.arange(0, d[-1], step)
    return np.c_[np.interp(t, d, p[:, 0]), np.interp(t, d, p[:, 1])]


def _turning(p, k):
    a = np.roll(p, k, 0); b = np.roll(p, -k, 0); v1 = p - a; v2 = b - p
    return np.abs(np.degrees(np.arctan2(v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0], (v1 * v2).sum(1))))


def real_corners(p: np.ndarray, eff_px: float, step: float, thr_deg: float = 32.0) -> list[int]:
    """Indices of real corners on a closed, evenly resampled contour p."""
    k = max(3, int(eff_px / step)); t = _turning(p, k); n = len(p); cand = []
    for i in np.argsort(-t):
        if t[i] < thr_deg: break
        if all(min(abs(i - j), n - abs(i - j)) > 2 * k for j in cand): cand.append(i)
    r = int(eff_px / step); keep = []
    for c in sorted(cand):
        A = p[[(c - r - j) % n for j in range(1, 2 * r)]]; B = p[[(c + r + j) % n for j in range(1, 2 * r)]]
        ra = np.linalg.svd(A - A.mean(0))[1][1] / np.sqrt(len(A)); rb = np.linalg.svd(B - B.mean(0))[1][1] / np.sqrt(len(B))
        dirn = lambda Q: np.arctan2(*(Q[-1] - Q[0])[::-1])
        bend = lambda Q: abs((dirn(Q[:len(Q) // 2 + 1]) - dirn(Q[len(Q) // 2:]) + np.pi) % (2 * np.pi) - np.pi)
        turn = abs((dirn(A) - dirn(B) + np.pi) % (2 * np.pi) - np.pi)
        # straight sides, sides keep their direction, and the sides themselves meet at a real angle
        if max(ra, rb) <= eff_px / 8 and max(bend(A), bend(B)) < 0.25 * turn and np.degrees(turn) >= thr_deg:
            keep.append(c)
    return keep


def rebuild_corners(p: np.ndarray, cs: list[int], eff_px: float, step: float) -> np.ndarray:
    """Replace the softened zone around each real corner by straight runs into the sides' intersection."""
    p = p.copy(); n = len(p); r = int(eff_px / step)
    for c in cs:
        A = p[[(c - r - j) % n for j in range(1, 2 * r)]]; B = p[[(c + r + j) % n for j in range(1, 2 * r)]]
        da = np.linalg.svd(A - A.mean(0))[2][0]; db = np.linalg.svd(B - B.mean(0))[2][0]; M = np.c_[da, -db]
        if abs(np.linalg.det(M)) < 0.15: continue
        tt = np.linalg.solve(M, B.mean(0) - A.mean(0)); v = A.mean(0) + tt[0] * da
        if np.linalg.norm(v - p[c]) > 0.9 * eff_px: continue
        for j in range(-r, r + 1):
            a = abs(j) / r; end = p[(c - r) % n] if j < 0 else p[(c + r) % n]; p[(c + j) % n] = v * (1 - a) + end * a
    return p


def _fit_seg(seg, closed, sig, tol):
    n = len(seg)
    if n < 5: return seg
    w = np.ones(n)
    if not closed: w[[0, -1]] = 1e3
    for mult in (1.0, .5, .25, .1, 0.0):
        try:
            tck, _ = splprep([seg[:, 0], seg[:, 1]], w=w, s=n * (sig * mult) ** 2, per=int(closed), k=3 if n > 8 else 1)
        except Exception:
            continue
        c = np.column_stack(splev(np.linspace(0, 1, max(n * 3, 20)), tck))
        if not closed: c[0], c[-1] = seg[0], seg[-1]
        if LineString(seg).hausdorff_distance(LineString(c)) <= tol: return c
    return seg


def smooth_ring(ring: np.ndarray, m: dict, step: float = 0.25) -> tuple[np.ndarray, int]:
    """Bounded smoothing of one closed contour using calibrate.metrics() output `m`."""
    p = resample(np.vstack([ring, ring[:1]]), step)[:-1]
    if len(p) < 8: return p, 0
    eff = m['effective_source_pixel_px']; cs = real_corners(p, eff, step)
    if cs: p = rebuild_corners(p, cs, eff, step)
    sig, tol = m['smoothing_sigma_px'], m['max_shift_px']
    if not cs: return _fit_seg(p, True, sig, tol), 0
    parts = []
    for a, b in zip(cs, cs[1:] + [cs[0] + len(p)]):
        parts.append(_fit_seg(np.vstack([p, p])[a:b + 1], False, sig, tol)[:-1])
    return np.vstack(parts), len(cs)


def mask_to_polygon(mask, m, scale=1, origin=(0, 0), small=False):
    """Mask -> shapely (Multi)Polygon with holes (even-odd), bounded-smoothed."""
    g = Polygon()
    for r in mask_rings(mask, scale, origin, sigma=0.9 if small else 1.4, min_area=0.3 if small else 2.0):
        sr, _ = smooth_ring(r, m)
        if len(sr) >= 3: g = g.symmetric_difference(Polygon(sr).buffer(0))
    return g


def drop_specks(geom, min_area: float, keep_whole_if_small: bool = True):
    """Remove raster specks from a part: islands and holes smaller than `min_area` (same units as geom).

    Why: tracing a part's colour region on low-resolution, compressed art leaves dozens of tiny islands
    (one run produced 71 islands, mostly < 0.3 mm^2, in a single SKIN layer) and pinholes. They are
    unprintable and noisy in CAD. A part that is itself tiny (a nose dot, an eye glint listed in the part
    spec) is kept: if every piece is below the threshold, the largest one survives.
    Suggested min_area: max(0.3 mm^2, (2 effective px)^2) - see calibrate.metrics.
    """
    from shapely.geometry import Polygon as _P, MultiPolygon as _MP
    from shapely.ops import unary_union as _uu
    polys = [p for p in getattr(geom, 'geoms', [geom]) if not p.is_empty]
    if not polys: return geom
    keep = []
    for p in polys:
        if p.area < min_area: continue
        holes = [h for h in p.interiors if _P(h).area >= min_area]
        keep.append(_P(p.exterior, holes))
    if not keep and keep_whole_if_small:
        big = max(polys, key=lambda q: q.area); keep = [_P(big.exterior)]
    return _uu(keep) if keep else _P()
