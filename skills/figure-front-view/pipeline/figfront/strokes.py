"""Thin drawn lines (eyebrows, lips, face lines) as constant-width strokes with round ends.

Why: a 1-2 px stroke traced as an outline comes out wavy or blob-shaped, and averaging it with its mirror
image shrinks it when the artwork draws left and right a little offset. A stroke has three intents only:
centre line (straight or circular), width, end caps. Fit those to the artwork pixels.
Centre line = mean y per grid column (face strokes are wide, not tall); a skeleton grows spurs at thick
stroke ends. Columns within half a stroke width of each end are caps, not centre line.
"""
from __future__ import annotations
import numpy as np
from shapely.geometry import LineString
from .geom import fit_circle
from .outline import Seg, polygon


def _centre_points(mask, X, Y, scale):
    cols = np.nonzero(mask.any(0))[0]
    P = np.array([[X[0, c], Y[mask[:, c], c].mean()] for c in cols])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    w0 = (mask.sum() / scale ** 2) / max(s[-1], 1e-6)
    return P[(s >= w0 / 2) & (s <= s[-1] - w0 / 2)]


def _with_caps(segs, ref_poly):
    """End caps are half circles; pick the turning direction that reproduces the buffered reference."""
    if polygon(segs).symmetric_difference(ref_poly).area > 0.1 * ref_poly.area:
        segs[1].sweep = -segs[1].sweep
        segs[3].sweep = -segs[3].sweep
    return polygon(segs), segs


def line_stroke(mask, X, Y, scale):
    """Straight stroke. mask on a `scale`x grid; X, Y = grid-pixel centres in image px."""
    cols = np.nonzero(mask.any(0))[0]
    P = np.array([[X[0, c], Y[mask[:, c], c].mean()] for c in cols])
    c0 = P.mean(0); d = np.linalg.svd(P - c0)[2][0]; d = d if d[0] > 0 else -d
    t = (P - c0) @ d; L = t.max() - t.min(); A = mask.sum() / scale ** 2
    q = 1 - np.pi / 4                                   # area = L*w - q*w^2 (L includes both round ends)
    w = (L - np.sqrt(max(L ** 2 - 4 * q * A, 0))) / (2 * q)
    e0 = c0 + (t.min() + w / 2) * d; e1 = c0 + (t.max() - w / 2) * d
    n = np.array([-d[1], d[0]]) * w / 2
    segs = [Seg("line", e0 + n, e1 + n), Seg("arc", e1 + n, e1 - n, e1, w / 2, -np.pi),
            Seg("line", e1 - n, e0 - n), Seg("arc", e0 - n, e0 + n, e0, w / 2, -np.pi)]
    poly, segs = _with_caps(segs, LineString([e0, e1]).buffer(w / 2, quad_segs=32))
    rms = float(np.sqrt(np.mean(((P - c0) @ np.array([-d[1], d[0]])) ** 2)))
    return poly, segs, dict(kind="line", width=float(w), length=float(L), R=float("inf"),
                            angle_deg=float(np.degrees(np.arctan2(d[1], d[0]))), centre_rms=rms)


def arc_stroke(mask, X, Y, scale, axis=None):
    """Circular-arc stroke. axis=x forces the circle centre onto that vertical (a stroke drawn on the axis)."""
    P = _centre_points(mask, X, Y, scale)
    if axis is not None:
        from scipy.optimize import least_squares
        res = lambda q: np.hypot(P[:, 0] - axis, P[:, 1] - q[0]) - q[1]
        ym = P[:, 1].mean()
        cand = [least_squares(res, [ym + s * 60, 60]).x for s in (-1, 1)]
        q = min(cand, key=lambda q: np.abs(res(q)).mean()); c = np.array([axis, q[0]]); R = abs(q[1])
    else:
        cx, cy, R, _ = fit_circle(P); c = np.array([cx, cy])
    ang = np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0]); am = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())
    rel = (ang - am + np.pi) % (2 * np.pi) - np.pi
    a0, a1 = am + rel.min(), am + rel.max()
    if axis is not None:
        h = 0.5 * (rel.max() - rel.min()); am = np.pi / 2 if c[1] < P[:, 1].mean() else -np.pi / 2
        a0, a1 = am - h, am + h
    Ls = R * (a1 - a0); A = mask.sum() / scale ** 2
    w = (-Ls + np.sqrt(Ls ** 2 + np.pi * A)) / (np.pi / 2)        # area = Ls*w + pi*w^2/4
    t = np.linspace(a0, a1, 200); centre = np.c_[c[0] + R * np.cos(t), c[1] + R * np.sin(t)]
    u = lambda a: np.r_[np.cos(a), np.sin(a)]
    so = Seg("arc", c + (R + w / 2) * u(a0), c + (R + w / 2) * u(a1), c, R + w / 2, a1 - a0)
    c1 = Seg("arc", so.b, c + (R - w / 2) * u(a1), centre[-1], w / 2, np.pi)
    si = Seg("arc", c1.b, c + (R - w / 2) * u(a0), c, R - w / 2, -(a1 - a0))
    c0 = Seg("arc", si.b, so.a, centre[0], w / 2, np.pi)
    poly, segs = _with_caps([so, c1, si, c0], LineString(centre).buffer(w / 2, quad_segs=32))
    return poly, segs, dict(kind="arc", centre=c.tolist(), R=float(R), width=float(w),
                            a0_deg=float(np.degrees(a0)), a1_deg=float(np.degrees(a1)),
                            centre_rms=float(np.std(np.hypot(*(P - c).T) - R)))
