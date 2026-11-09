"""Regularity-aware edge fitting: choose the SIMPLEST curve that stays faithful to the trace.

Why: in the first project 13 of ~27 review rounds were the user asking for edge regularity by hand
("straight lines", "continuous curve", "corner sharp", "the chin is a circle", "keep the small curve").
Perception studies on vectorizing artist-drawn rasters (Hoshyari et al., SIGGRAPH 2018) show viewers
expect: corners only where the boundary really breaks, and between corners the simplest continuous curve
that captures regularities (lines, arcs). Beautification systems (ShipShape, Fišer et al. 2015/16) add
snapping: arc centres, tangent alignment, reflection symmetry.

Rule implemented here, per boundary run between real corners:
    candidates  line  <  circular arc  <  cubic spline with 0..K interior knots   (simplicity order)
    accept the simplest whose max deviation from the trace <= reg_tol
reg_tol defaults to 0.5 effective pixel: above the sub-pixel precision limit (0.25 eff px, see calibrate)
so drawing jitter is absorbed, below the size of real drawn features (~1 eff px: the notch between two
fists, a 0.35 mm bulge), so those survive as extra spline knots.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from scipy.interpolate import LSQUnivariateSpline
from shapely.geometry import LineString, Polygon
from .contours import resample, real_corners, rebuild_corners
from .geom import fit_circle


@dataclass
class Edge:
    kind: str                    # 'line' | 'arc' | 'spline'
    pts: np.ndarray              # regularized polyline (dense)
    dev: float                   # max deviation from the trace (px)
    params: dict = field(default_factory=dict)


def _maxdev(a, b):
    return LineString(a).hausdorff_distance(LineString(b))


def fit_line(seg):
    c = seg.mean(0); d = np.linalg.svd(seg - c)[2][0]; t = (seg - c) @ d
    out = np.c_[c + np.outer(np.linspace(t.min(), t.max(), max(len(seg), 2)), d)]
    out[0], out[-1] = seg[0], seg[-1]                       # keep the corner points exactly
    return out, dict(point=c.tolist(), dir=d.tolist())


def fit_arc(seg, max_radius):
    cx, cy, R, _ = fit_circle(seg)
    if R > max_radius: return None, None
    a0 = np.arctan2(seg[0, 1] - cy, seg[0, 0] - cx); a1 = np.arctan2(seg[-1, 1] - cy, seg[-1, 0] - cx)
    mid = np.arctan2(seg[len(seg) // 2, 1] - cy, seg[len(seg) // 2, 0] - cx)
    da = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
    if not (min(a0, a0 + da) - 1e-9 <= (mid if abs(da) > 0 else a0) <= max(a0, a0 + da) + 1e-9):
        da = da - np.sign(da) * 2 * np.pi                   # take the long way round if the middle says so
    a = np.linspace(a0, a0 + da, max(len(seg), 8)); out = np.c_[cx + R * np.cos(a), cy + R * np.sin(a)]
    out[0], out[-1] = seg[0], seg[-1]
    return out, dict(center=[cx, cy], radius=R, sweep_deg=float(np.degrees(da)))


def fit_spline(seg, knots, closed=False):
    a = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
    if a[-1] <= 0 or len(seg) < 8: return seg, {}
    kn = np.linspace(a[0], a[-1], knots + 2)[1:-1]
    w = np.ones(len(a))
    if not closed: w[[0, -1]] = 1e4
    try:
        sx = LSQUnivariateSpline(a, seg[:, 0], kn, w=w, k=3); sy = LSQUnivariateSpline(a, seg[:, 1], kn, w=w, k=3)
    except Exception:
        return seg, {}
    out = np.c_[sx(a), sy(a)]
    if not closed: out[0], out[-1] = seg[0], seg[-1]
    return out, dict(interior_knots=knots)


def _signed_residual(seg, fit):
    """signed distance of trace points to the fitted polyline (sign from the fit's left normal)."""
    from shapely.geometry import Point
    L = LineString(fit); out = np.empty(len(seg))
    for i, q in enumerate(seg):
        s = L.project(Point(q)); a = np.array(L.interpolate(s).coords[0]); b = np.array(L.interpolate(min(s + 0.05, L.length)).coords[0])
        t = b - a; n = np.array([-t[1], t[0]]); out[i] = np.sign((q - a) @ n) * np.linalg.norm(q - a)
    return out


def _coherent_ok(seg, fit, run_len, coherent_tol):
    """Real drawn features bend one way over several pixels; jitter alternates quickly. Reject a fit that
    leaves a one-signed residual longer than run_len with a peak above coherent_tol (= a flattened feature)."""
    r = _signed_residual(seg, fit)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
    sign = np.sign(r); start = 0
    for i in range(1, len(r) + 1):
        if i == len(r) or sign[i] != sign[start]:
            if s[i - 1] - s[start] >= run_len and np.abs(r[start:i]).max() > coherent_tol: return False
            start = i
    return True


def _piecewise(seg, tol, max_radius, step_pts):
    """Fewest line/arc pieces (cost line=1, arc=2) covering the run within tol: dynamic programming over
    breakpoints every `step_pts` samples. Returns (cost, [(kind, pts, params), ...]) or (inf, None)."""
    idx = list(range(0, len(seg) - 1, step_pts)) + [len(seg) - 1]; n = len(idx)
    best = [np.inf] * n; prev = [None] * n; best[0] = 0
    for j in range(1, n):
        for i in range(j - 1, -1, -1):
            if best[i] == np.inf: continue
            sub = seg[idx[i]:idx[j] + 1]
            if len(sub) < 3: continue
            p, prm = fit_line(sub); c = 1
            if _maxdev(sub, p) > tol:
                p, prm = fit_arc(sub, max_radius); c = 2
                if p is None or _maxdev(sub, p) > tol: continue
            if best[i] + c < best[j]: best[j] = best[i] + c; prev[j] = (i, 'line' if c == 1 else 'arc', p, prm)
    if best[-1] == np.inf: return np.inf, None
    pieces = []; j = n - 1
    while j > 0:
        i, k, p, prm = prev[j]; pieces.append((k, p, prm)); j = i
    return best[-1], pieces[::-1]


def regularize_run(seg, reg_tol, max_radius=1e3, max_knots=24, closed=False, allow=('line', 'arc', 'spline'),
                   coherent_tol=None, coherent_len=None, piecewise=True) -> Edge:
    """Simplest faithful curve for one boundary run (open, between corners; or closed without corners).
    Order of preference: one line < one arc < a few line/arc pieces (tangent joins, e.g. straight side
    blending into a round end) < one cubic spline with the fewest knots. Every candidate must stay within
    reg_tol of the trace AND must not flatten a coherent drawn feature (see _coherent_ok)."""
    ct = coherent_tol if coherent_tol is not None else reg_tol / 2
    cl = coherent_len if coherent_len is not None else 6 * reg_tol
    ok = lambda p: _maxdev(seg, p) <= reg_tol and _coherent_ok(seg, p, cl, ct)
    if 'line' in allow and not closed:
        p, prm = fit_line(seg)
        if ok(p): return Edge('line', p, _maxdev(seg, p), prm)
    if 'arc' in allow:
        if closed:
            cx, cy, R, _ = fit_circle(seg)
            t = np.linspace(0, 2 * np.pi, max(len(seg), 64), endpoint=False); p = np.c_[cx + R * np.cos(t), cy + R * np.sin(t)]
            prm = dict(center=[cx, cy], radius=R, sweep_deg=360.0)
        else:
            p, prm = fit_arc(seg, max_radius)
        if p is not None and ok(p): return Edge('arc', p, _maxdev(seg, p), prm)
    spline = None
    for k in range(0, max_knots + 1):
        p, prm = fit_spline(seg, k, closed)
        if not prm: break
        spline = Edge('spline', p, _maxdev(seg, p), prm)
        if ok(p): break
    if piecewise and not closed and len(seg) > 12:
        cost, pieces = _piecewise(seg, reg_tol, max_radius, max(2, len(seg) // 40))
        spline_cost = (spline.params.get('interior_knots', max_knots) + 3) if spline else np.inf
        if pieces and cost < spline_cost:
            pts = np.vstack([pc[1][:-1] for pc in pieces] + [pieces[-1][1][-1:]])
            if ok(pts):
                return Edge('pieces', pts, _maxdev(seg, pts), dict(pieces=[(k, prm) for k, _, prm in pieces]))
    return spline or Edge('spline', seg, 0.0, {})


def regularize_ring(ring: np.ndarray, m: dict, reg_tol_eff: float = 0.5, step: float = 0.25, **kw):
    """Closed contour -> list of Edge (corners kept sharp and rebuilt) and the joined polygon points.
    m: calibrate.metrics() output."""
    eff = m['effective_source_pixel_px']; reg_tol = reg_tol_eff * eff
    p = resample(np.vstack([ring, ring[:1]]), step)[:-1]
    cs = real_corners(p, eff, step)
    if cs: p = rebuild_corners(p, cs, eff, step)
    if not cs:
        e = regularize_run(p, reg_tol, closed=True, **kw); return [e], e.pts
    edges = []
    for a, b in zip(cs, cs[1:] + [cs[0] + len(p)]):
        edges.append(regularize_run(np.vstack([p, p])[a:b + 1], reg_tol, **kw))
    return edges, np.vstack([e.pts[:-1] for e in edges])


def regularize_polygon(poly: Polygon, m: dict, **kw):
    """Shapely polygon -> (regularized polygon, edge report list)."""
    report = []
    ext_edges, ext = regularize_ring(np.array(poly.exterior.coords)[:-1], m, **kw)
    holes = []
    for h in poly.interiors:
        he, hp = regularize_ring(np.array(h.coords)[:-1], m, **kw); holes.append(hp); report += [('hole', e) for e in he]
    report = [('outer', e) for e in ext_edges] + report
    return Polygon(ext, holes).buffer(0), report


def mirror_pairs(edges_by_part: dict, axis_x: float, tol: float):
    """Find edge pairs that are mirror images about x=axis_x within tol (symmetry snapping candidates)."""
    flat = [(k, i, e) for k, es in edges_by_part.items() for i, (_, e) in enumerate(es)]
    pairs = []
    for a in range(len(flat)):
        ka, ia, ea = flat[a]; ma = ea.pts.copy(); ma[:, 0] = 2 * axis_x - ma[:, 0]
        for b in range(a + 1, len(flat)):
            kb, ib, eb = flat[b]
            if ea.kind != eb.kind: continue
            if _maxdev(ma, eb.pts) <= tol: pairs.append(((ka, ia), (kb, ib)))
    return pairs
