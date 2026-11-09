"""Outline builder: dense ring -> closed chain of LINE and ARC segments.

Corners stay sharp (figfront real-corner test + rebuild). Between corners each run gets the simplest
faithful curve from figfront.regularize (line < arc < spline, no line/arc pieces: their joins are not
tangent). Curves are then replaced by biarc chains: arcs that meet tangent (G1), which is what CAD
sketches and the DXF ARC entity hold. For a mirror-symmetric part only the right half is fitted; it is
mirrored onto the left, and runs that cross the axis smoothly get a tangent perpendicular to the axis.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from .contours import resample, real_corners, rebuild_corners
from .regularize import regularize_run


@dataclass
class Seg:
    kind: str                 # 'line' | 'arc'
    a: np.ndarray             # start point
    b: np.ndarray             # end point
    c: np.ndarray | None = None   # arc centre
    r: float = 0.0            # arc radius
    sweep: float = 0.0        # signed sweep (rad), positive = counter-clockwise in x-right/y-down axes as math angle

    def points(self, step=0.1):
        if self.kind == "line":
            n = max(2, int(np.linalg.norm(self.b - self.a) / step) + 1)
            return self.a + np.linspace(0, 1, n)[:, None] * (self.b - self.a)
        a0 = np.arctan2(*(self.a - self.c)[::-1])
        n = max(3, int(abs(self.sweep) * self.r / step) + 1)
        t = a0 + np.linspace(0, self.sweep, n)
        p = self.c + self.r * np.c_[np.cos(t), np.sin(t)]
        p[0], p[-1] = self.a, self.b
        return p

    def mirrored(self, axis):
        f = lambda p: None if p is None else np.array([2 * axis - p[0], p[1]])
        return Seg(self.kind, f(self.b), f(self.a), f(self.c), self.r, self.sweep)


def _arc_from(A, T, B):
    """Arc starting at A with unit tangent T, ending at B. None if B lies on the tangent line."""
    n = np.array([-T[1], T[0]]); v = B - A; den = 2 * (n @ v)
    if abs(den) < 1e-9 * max(1.0, v @ v): return Seg("line", A.copy(), B.copy())
    r = (v @ v) / den; c = A + r * n
    a0 = np.arctan2(*(A - c)[::-1]); a1 = np.arctan2(*(B - c)[::-1])
    sw = (a1 - a0) % (2 * np.pi)
    if r < 0: sw -= 2 * np.pi          # centre on the right of the tangent: clockwise in math angles
    return Seg("arc", A.copy(), B.copy(), c, abs(r), sw)


def biarc(P0, T0, P1, T1):
    v = P1 - P0; t = T0 + T1; k = 1 - T0 @ T1
    if np.linalg.norm(v) < 1e-9: return []
    if k < 1e-9:
        vt = v @ T1
        if abs(vt) < 1e-9: return None
        d = (v @ v) / (4 * vt)
    else:
        vt = v @ t; d = (-vt + np.sqrt(vt * vt + 2 * k * (v @ v))) / (2 * k)
    if not np.isfinite(d) or d <= 0: return None
    Pm = 0.5 * ((P0 + d * T0) + (P1 - d * T1))
    return [_arc_from(P0, T0, Pm), _arc_from(P1, -T1, Pm)]


def _flip(s):
    if s.kind == "line": return Seg("line", s.b, s.a)
    return Seg("arc", s.b, s.a, s.c, s.r, -s.sweep)


def _dev(segs, pts):
    """max distance of pts to the chain"""
    best = np.full(len(pts), np.inf)
    for s in segs:
        if s.kind == "line":
            ab = s.b - s.a; L = ab @ ab
            u = np.clip(((pts - s.a) @ ab) / max(L, 1e-12), 0, 1)
            d = np.linalg.norm(pts - (s.a + u[:, None] * ab), axis=1)
        else:
            q = s.points(0.05); d = np.min(np.linalg.norm(pts[:, None] - q[None], axis=2), axis=1)
        best = np.minimum(best, d)
    return float(best.max())


def _tangent(p, i, w=6):
    """unit tangent at sample i from a local quadratic fit over +-w samples"""
    j0, j1 = max(i - w, 0), min(i + w, len(p) - 1)
    q = p[j0:j1 + 1]
    if len(q) < 3:
        t = p[min(i + 1, len(p) - 1)] - p[max(i - 1, 0)]; return t / (np.linalg.norm(t) + 1e-12)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))]; s0 = s[i - j0]
    cx = np.polyfit(s - s0, q[:, 0], 2); cy = np.polyfit(s - s0, q[:, 1], 2)
    t = np.array([cx[1], cy[1]]); return t / (np.linalg.norm(t) + 1e-12)


def _dev_each(segs, pts):
    best = np.full(len(pts), np.inf)
    for s in segs:
        if s.kind == "line":
            ab = s.b - s.a; L = ab @ ab
            u = np.clip(((pts - s.a) @ ab) / max(L, 1e-12), 0, 1)
            d = np.linalg.norm(pts - (s.a + u[:, None] * ab), axis=1)
        else:
            q = s.points(0.05); d = np.min(np.linalg.norm(pts[:, None] - q[None], axis=2), axis=1)
        best = np.minimum(best, d)
    return best


def fit_biarcs(p, T0, T1, tol, w=6, depth=0):
    """G1 chain of arcs fitted to the dense trace p with given end tangents; split at the worst point."""
    if len(p) < 3: return [Seg("line", p[0].copy(), p[-1].copy())]
    ba = biarc(p[0], T0, p[-1], T1)
    if ba is not None:
        ba = [ba[0], _flip(ba[1])]
        d = _dev_each(ba, p)
        if d.max() <= tol or depth > 14 or len(p) < 2 * w + 3: return ba
        k = int(np.clip(np.argmax(d), w, len(p) - 1 - w))
    else:
        k = len(p) // 2
    Tk = _tangent(p, k, w)
    return fit_biarcs(p[:k + 1], T0, Tk, tol, w, depth + 1) + fit_biarcs(p[k:], Tk, T1, tol, w, depth + 1)


def _straighten(segs, max_half_sweep_deg=0.5):
    """Near-flat arcs -> lines (the tangent break is at most half the sweep), then merge collinear lines."""
    out = []
    for s in segs:
        if s.kind == "arc" and np.degrees(abs(s.sweep)) / 2 < max_half_sweep_deg: s = Seg("line", s.a, s.b)
        if out and s.kind == "line" and out[-1].kind == "line":
            u = out[-1].b - out[-1].a; v = s.b - s.a
            ang = np.degrees(abs(np.arctan2(u[0] * v[1] - u[1] * v[0], u @ v)))
            if ang < 2 * max_half_sweep_deg:
                out[-1] = Seg("line", out[-1].a, s.b); continue
        out.append(s)
    return out


def _single_arc(p, T0, T1):
    """One arc through both run ends. A forced horizontal end tangent (smooth axis crossing) puts the
    centre on the vertical through that end; otherwise the arc passes through the run's middle sample."""
    A, B = p[0], p[-1]
    if T0 is not None or T1 is not None:
        E, O_ = (A, B) if T0 is not None else (B, A)
        bx = O_[0] - E[0]; den = 2 * (E[1] - O_[1])
        if abs(den) < 1e-9: return None
        cy = (E[1] ** 2 - bx ** 2 - O_[1] ** 2) / den
        c = np.array([E[0], cy])
    else:
        M = p[len(p) // 2]
        a = np.array([[2 * (M[0] - A[0]), 2 * (M[1] - A[1])], [2 * (B[0] - A[0]), 2 * (B[1] - A[1])]])
        if abs(np.linalg.det(a)) < 1e-9: return None
        c = np.linalg.solve(a, [M @ M - A @ A, B @ B - A @ A])
    r = np.linalg.norm(A - c)
    a0 = np.arctan2(A[1] - c[1], A[0] - c[0]); a1 = np.arctan2(B[1] - c[1], B[0] - c[0])
    am = np.arctan2(*(p[len(p) // 2] - c)[::-1])
    sw = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
    rel = (am - a0 + np.pi) % (2 * np.pi) - np.pi
    if np.sign(rel) != np.sign(sw) or abs(rel) > abs(sw):    # the middle says: the long way round
        sw = sw - np.sign(sw) * 2 * np.pi
    return Seg("arc", A.copy(), B.copy(), c, r, sw)


def _run_to_segs(seg_pts, reg_tol, tol, T0=None, T1=None, w=10, smooth_px=1.5):
    """line or single arc when it is faithful (figfront.regularize), else a biarc chain on the trace."""
    e = regularize_run(seg_pts, reg_tol, piecewise=False, allow=("line", "arc"), max_knots=0)
    if e.kind == "line" and T0 is None and T1 is None:
        return [Seg("line", seg_pts[0].copy(), seg_pts[-1].copy())], e
    if e.kind == "arc":
        one = _single_arc(seg_pts, T0, T1)
        if one is not None and _dev_each([one], seg_pts).max() <= reg_tol:
            e.kind = "arc"
            return [one], e
        q = e.pts
    else:
        # light smoothing (sigma = smooth_px along the curve) so tangents and splits ignore drawing jitter;
        # the end points stay fixed
        from scipy.ndimage import gaussian_filter1d
        sg = max(smooth_px / 0.25, 1e-3); pad = int(3 * sg) + 1
        ext = np.vstack([2 * seg_pts[0] - seg_pts[pad:0:-1], seg_pts, 2 * seg_pts[-1] - seg_pts[-2:-pad - 2:-1]])
        q = gaussian_filter1d(ext, sg, axis=0, mode="nearest")[pad:pad + len(seg_pts)]
        q[0], q[-1] = seg_pts[0], seg_pts[-1]
    T0 = _tangent(q, 0, w) if T0 is None else T0
    T1 = _tangent(q, len(q) - 1, w) if T1 is None else T1
    segs = _straighten(fit_biarcs(q, T0, T1, tol, w))
    arcs = [x for x in segs if x.kind == "arc"]
    if all(x.kind == "line" for x in segs) and len(segs) == 1: kind = "line"
    elif len(arcs) == len(segs) and all(np.linalg.norm(x.c - arcs[0].c) < 0.03 * arcs[0].r + 0.5 and abs(x.r - arcs[0].r) < 0.02 * arcs[0].r + 0.3 for x in arcs): kind = "arc"
    else: kind = "curve"
    e.kind = kind
    return segs, e


def _zone_kw(run, fine_zone, reg_tol, biarc_tol):
    """runs mostly inside the fine zone follow the trace at the fidelity budget, without smoothing"""
    if fine_zone is None: return reg_tol, biarc_tol, {}
    inside = np.mean((run[:, 1] >= fine_zone[0]) & (run[:, 1] <= fine_zone[1]))
    return (reg_tol, biarc_tol, {}) if inside < 0.5 else (reg_tol / 2, biarc_tol / 2, dict(smooth_px=0.0))


def build(ring, m, axis=None, reg_tol_eff=0.5, biarc_tol_eff=0.5, step=0.25, corner_deg=32.0, corners_at=None,
          fine_zone=None, fine_scale=0.5):
    """fine_zone=(y0, y1): also find corners at fine_scale x the corner scale there (short drawn steps)."""
    """ring: Nx2 closed contour (px). Returns (segments in ring order, report rows).
    biarc chains stay within biarc_tol_eff effective pixels of the trace (the raster-defect budget)."""
    eff = m["effective_source_pixel_px"]; reg_tol = reg_tol_eff * eff; biarc_tol = biarc_tol_eff * eff
    p = resample(np.vstack([ring, ring[:1]]), step)[:-1]
    if Polygon(p).exterior.is_ccw: p = p[::-1]                # one orientation for every ring
    cs = real_corners(p, eff, step, thr_deg=corner_deg)
    if cs: p = rebuild_corners(p, cs, eff, step)
    if fine_zone is not None:
        # short drawn steps (2-5 px sides): simplify the trace, keep vertices that turn >= 40 deg between
        # two sides >= 1.2 px; the corner goes to the intersection of the two side lines
        from shapely.geometry import LineString as _LS
        n0 = len(p); dp = np.array(_LS(np.vstack([p, p[:1]])).simplify(0.4).coords)[:-1]
        kf = 4; add = []
        for j in range(len(dp)):
            A, V, B = dp[j - 1], dp[j], dp[(j + 1) % len(dp)]
            if not (fine_zone[0] <= V[1] <= fine_zone[1]): continue
            u, w = V - A, B - V
            if min(np.linalg.norm(u), np.linalg.norm(w)) < 1.2: continue
            ang = np.degrees(abs(np.arctan2(u[0] * w[1] - u[1] * w[0], u @ w)))
            if ang < 40: continue
            i = int(np.linalg.norm(p - V, axis=1).argmin())
            if any(min(abs(i - c), n0 - abs(i - c)) <= kf for c in cs + add): continue
            add.append(i)
            Mx = np.c_[u, -w]
            if abs(np.linalg.det(Mx)) > 1e-6:
                t_ = np.linalg.solve(Mx, B - A)
                X_ = A + t_[0] * u
                if np.linalg.norm(X_ - V) < 1.0: p[i] = X_
        cs = sorted(cs + add)
    for q in (corners_at if corners_at is not None else []):   # known exact vertices (exact geometry)
        d = np.linalg.norm(p - np.asarray(q), axis=1); i = int(d.argmin())
        if d[i] < 1.0:
            p[i] = q; cs = sorted(set(c for c in cs if min(abs(c - i), n0 - abs(c - i)) > 2) | {i}) if (n0 := len(p)) else cs
    n = len(p); report = []
    if axis is None:
        if not cs:
            # closed smooth curve: split into two runs at opposite points, tangents shared
            i0, i1 = 0, n // 2
            T0, T1 = _tangent(np.vstack([p[-3:], p[:4]]), 3), _tangent(p, i1)
            s1, e1 = _run_to_segs(p[i0:i1 + 1], reg_tol, biarc_tol, T0, T1)
            s2, e2 = _run_to_segs(np.vstack([p[i1:], p[:1]]), reg_tol, biarc_tol, T1, T0)
            return s1 + s2, [("closed", e1.kind, len(s1) + len(s2), max(e1.dev, e2.dev), s1 + s2)], polygon(s1 + s2)
        segs = []
        for a, b in zip(cs, cs[1:] + [cs[0] + n]):
            run = np.vstack([p, p])[a:b + 1]; rt, bt, zk = _zone_kw(run, fine_zone, reg_tol, biarc_tol)
            s, e = _run_to_segs(run, rt, bt, **zk)
            segs += s; report.append(("run", e.kind, len(s), e.dev, s))
        return segs, report, polygon(segs)
    # ---- symmetric: split points = corners on the right half + axis crossings ----
    sx = np.sign(p[:, 0] - axis); sx[sx == 0] = 1
    cross = [i for i in range(n) if sx[i] != sx[(i + 1) % n]]
    pts = p.copy(); kinds = {}
    for i in cross:                                             # move the crossing sample onto the axis
        j = (i + 1) % n; t = (axis - p[i, 0]) / (p[j, 0] - p[i, 0]); pts[i] = p[i] + t * (p[j] - p[i]); pts[i, 0] = axis
        near = [c for c in cs if min(abs(c - i), n - abs(c - i)) <= int(1.5 * eff / step)]
        kinds[i] = "corner" if near else "smooth"
    cs_r = [c for c in cs if p[c, 0] > axis + 0.5 and all(min(abs(c - i), n - abs(c - i)) > int(1.5 * eff / step) for i in cross)]
    for c in cs_r: kinds[c] = "corner"
    split = sorted(kinds)
    if not split or not cross:
        return build(ring, m, None, reg_tol_eff, biarc_tol_eff, step, corner_deg, corners_at, fine_zone, fine_scale)
    k0 = split.index(cross[0]); split = split[k0:] + split[:k0]          # start at an axis crossing
    right = []
    for a, b in zip(split, split[1:] + split[:1]):
        b = a + (b - a) % n                         # forward along the ring, wrapping once
        idx = np.arange(a, b + 1) % n
        run = pts[idx]
        if np.median(run[:, 0]) < axis: continue
        T0 = np.array([1.0, 0.0]) * np.sign(run[min(3, len(run) - 1), 0] - axis) if kinds[a] == "smooth" and a in cross else None
        T1 = np.array([-1.0, 0.0]) * np.sign(run[max(-4, -len(run)), 0] - axis) if kinds[b % n] == "smooth" and (b % n) in cross else None
        if T0 is not None and abs(run[min(3, len(run) - 1), 0] - axis) < 1e-9: T0 = None
        rt, bt, zk = _zone_kw(run, fine_zone, reg_tol, biarc_tol)
        s, e = _run_to_segs(run, rt, bt, T0, T1, **zk)
        right.append(s); report.append(("run", e.kind, len(s), e.dev, s))
    # right half in ring order (consecutive groups meet along the axis), closed and mirrored
    chain = [s for r in right for s in r]
    for sg in chain:                               # ends that belong on the axis sit exactly on it
        for pnt in (sg.a, sg.b):
            if abs(pnt[0] - axis) < 0.25: pnt[0] = axis
    half = polygon(chain, open_chain=True)
    halves = getattr(half, "geoms", [half])
    mir = [Polygon([(2 * axis - x, y) for x, y in q.exterior.coords]) for q in halves]
    e = 0.02                                       # weld the two halves along the axis (no slit)
    poly = unary_union([q.buffer(e) for q in [*halves, *mir]]).buffer(-e)
    return chain + [s.mirrored(axis) for s in reversed(chain)], report, poly


def polygon(segs, step=0.1, open_chain=False):
    """Polygon from a closed segment loop, or from an open chain closed by a straight line (half part)."""
    pts = np.vstack([s.points(step)[:-1] for s in segs] + ([segs[-1].b[None]] if open_chain else []))
    return Polygon(pts).buffer(0)


def build_geom(g, m, axis=None, **kw):
    """shapely (Multi)Polygon -> (polygon rebuilt from segments, {ring_id: segments}, report)."""
    polys, rings, rep = [], [], []
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        # a part fully on one side of the axis is mirrored as a whole by the caller
        ax = axis if axis is not None and p.bounds[0] < axis < p.bounds[2] else None
        so, r, poly = build(np.array(p.exterior.coords)[:-1], m, ax, **kw); rep += [("outer",) + x for x in r]
        holes = []
        for h in p.interiors:
            hax = ax if ax is not None and h.bounds[0] < axis < h.bounds[2] else None
            sh, r, hp = build(np.array(h.coords)[:-1], m, hax, **kw); rep += [("hole",) + x for x in r]
            holes.append(sh); poly = poly.difference(hp)
        rings.append((so, holes))
        polys.append(poly)
    return unary_union(polys), rings, rep


def symmetric_part(g, m, axis, **kw):
    """Mirror-symmetric part from a (roughly symmetric) traced shape: pieces crossing the axis are fitted on
    the right half and mirrored; pieces wholly right of the axis are fitted and copied mirrored; pieces left
    of the axis are dropped (they are the mirror copies). The halves are welded and zero-area slits along the
    axis removed. Returns (polygon, rings [(outer segs, [hole segs])], report)."""
    from shapely import affinity
    out, rings, rep = [], [], []
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        x0, _, x1, _ = p.bounds
        if x1 <= axis: continue
        if x0 < axis < x1:
            poly, rg, r = build_geom(p, m, axis, **kw); out.append(poly); rings += rg; rep += r
        else:
            poly, rg, r = build_geom(p, m, None, **kw); rep += r
            out += [poly, affinity.scale(poly, -1, 1, origin=(axis, 0))]
            rings += rg + [([s.mirrored(axis) for s in reversed(so)], [[s.mirrored(axis) for s in reversed(h)] for h in hs])
                           for so, hs in rg]
    gg = unary_union([q.buffer(1e-3) for q in out]).buffer(-1e-3)
    gg = unary_union([Polygon(q.exterior, [h for h in q.interiors if Polygon(h).area > 0.5])
                      for q in getattr(gg, "geoms", [gg])])
    rings = [(so, [h for h in hs if polygon(h).area > 0.5]) for so, hs in rings]
    return gg, rings, rep


def tangent_breaks(segs):
    """angle (deg) between consecutive segments at every shared end point; real corners are large"""
    out = []
    for a, b in zip(segs, segs[1:] + segs[:1]):
        if np.linalg.norm(a.b - b.a) > 1e-6: continue
        pa, pb = a.points(0.01), b.points(0.01)
        ta, tb = pa[-1] - pa[-2], pb[1] - pb[0]
        out.append(float(np.degrees(np.arccos(np.clip(ta @ tb / (np.linalg.norm(ta) * np.linalg.norm(tb)), -1, 1)))))
    return out


# public names for project code that splices or bridges segment chains (method.md, "Scope-safe edits")
arc_from = _arc_from
flip = _flip
single_arc = _single_arc
