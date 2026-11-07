"""Exact shapes for parts that are geometric on the REAL object (check the print photo first).

Use exact geometry only where the print shows it (smooth oval arm ring, straight jacket sides, straight
V cravat, circular chin). Everything else keeps its traced outline.
"""
from __future__ import annotations
import numpy as np
from shapely.geometry import Polygon, Point, box, LineString
from shapely import affinity
from shapely.ops import unary_union
from scipy.interpolate import LSQUnivariateSpline
from scipy.optimize import least_squares


def ellipse(cx, cy, rx, ry, n=256, tilt_deg=0.0):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    e = Polygon(np.c_[cx + rx * np.cos(t), cy + ry * np.sin(t)])
    return affinity.rotate(e, tilt_deg, origin=(cx, cy)) if tilt_deg else e


def oval_ring(cx, cy, rx, ry, icx, icy, irx, iry):
    """ONE ring = outer oval minus inner oval (e.g. arms that loop round the body). Do not split it into
    two mirrored loops unless the object really is two parts."""
    return ellipse(cx, cy, rx, ry).difference(ellipse(icx, icy, irx, iry))


def fit_circle(pts: np.ndarray):
    """Algebraic least-squares circle -> (cx, cy, R, rms)."""
    A = np.c_[2 * pts, np.ones(len(pts))]; cx, cy, k = np.linalg.lstsq(A, (pts ** 2).sum(1), rcond=None)[0]
    R = float(np.sqrt(k + cx ** 2 + cy ** 2)); res = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - R
    return float(cx), float(cy), R, float(np.sqrt(np.mean(res ** 2)))


def straight_v(xc, top, w_at_ref, y_ref, w_at_v, y_v, y_tip):
    """Straight-line V (shirt/tie): sides run straight from (half-width w_at_ref at y_ref) to (w_at_v at
    y_v), extended up to `top`; then two straight edges to the tip. y grows downward."""
    k = (w_at_v - w_at_ref) / (y_v - y_ref); wtop = w_at_ref + k * (top - y_ref)
    return Polygon([(xc - wtop, top), (xc + wtop, top), (xc + w_at_v, y_v), (xc, y_tip), (xc - w_at_v, y_v)])


def flat_with_bulge(x, y_flat, depth, xc, half_width):
    """Edge that is flat except for a broad, round-bottomed bulge (e.g. top of the gap between legs)."""
    u = np.clip((np.asarray(x, float) - xc) / half_width, -1, 1); return y_flat + depth * (1 - u ** 4) ** 2


def fit_flat_with_bulge(pts, x_center_guess):
    q = least_squares(lambda q: flat_with_bulge(pts[:, 0], *q) - pts[:, 1],
                      [np.median(pts[np.abs(pts[:, 0] - x_center_guess) > 20, 1]), 1.3, x_center_guess, 14.0],
                      loss='soft_l1', f_scale=0.3).x
    return dict(y_flat=q[0], depth=q[1], xc=q[2], half_width=q[3])


def face_shape(A, cy, top_h, chin_h, xc, jaw_n=2.47, top_n=2.6, n=480):
    """Normal face outline: rounded-square forehead/temples above the eye line `cy`, superellipse jaw
    below. Use it for the part HIDDEN under hair/helmet; the visible chin follows the image."""
    t = np.linspace(0, 2 * np.pi, n, endpoint=False); c, s = np.cos(t), np.sin(t); up = s < 0
    x = np.where(up, A * np.sign(c) * np.abs(c) ** (2 / top_n), A * np.sign(c) * np.abs(c) ** (2 / jaw_n))
    y = np.where(up, cy + top_h * np.sign(s) * np.abs(s) ** (2 / top_n), cy + chin_h * s)
    return Polygon(np.c_[x + xc, y]).buffer(0)


def join_circles_y(c1, c2):
    """y where two circles (cx,cy,R) have the same half-width -> seamless join between two outlines."""
    (_, y1, r1), (_, y2, r2) = c1, c2
    return (r2 ** 2 - r1 ** 2 + y1 ** 2 - y2 ** 2) / (2 * (y1 - y2))


def corner_from_tangent(line_x, curve_pts):
    """Sharp concave/convex corner: extend the straight tangent of `curve_pts` until it meets x=line_x."""
    d = np.linalg.svd(curve_pts - curve_pts.mean(0))[2][0]
    t = (line_x - curve_pts.mean(0)[0]) / d[0]; return curve_pts.mean(0) + t * d


def spline_edge(seg: np.ndarray, n_interior_knots: int = 4) -> np.ndarray:
    """One continuous curve through a long edge (e.g. outer leg from waist corner to ankle), ends pinned."""
    a = np.r_[0, np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))]
    knots = np.linspace(a[0], a[-1], n_interior_knots + 2)[1:-1]; w = np.ones(len(a)); w[[0, -1]] = 1e4
    sx = LSQUnivariateSpline(a, seg[:, 0], knots, w=w, k=3); sy = LSQUnivariateSpline(a, seg[:, 1], knots, w=w, k=3)
    out = np.c_[sx(a), sy(a)]; out[0], out[-1] = seg[0], seg[-1]; return out


def poly_curve_fit(y, x, deg=2):
    """x(y) polynomial for near-vertical edges (inner leg edges)."""
    return np.polyfit(y, x, deg)
