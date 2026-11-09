import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely import affinity
from figfront import outline, strokes, pixels, checks

M = dict(effective_source_pixel_px=1.5)
rng = np.random.default_rng(3)


def ring_of(g, step=0.25, noise=0.03):
    r = np.array(Polygon(g.exterior).segmentize(step).exterior.coords)[:-1]
    return r + rng.normal(0, noise, r.shape)


def tangent_breaks(segs):
    out = []
    for a, b in zip(segs, segs[1:] + segs[:1]):
        if np.linalg.norm(a.b - b.a) > 1e-6: continue
        pa, pb = a.points(0.01), b.points(0.01)
        ta, tb = pa[-1] - pa[-2], pb[1] - pb[0]
        out.append(np.degrees(np.arccos(np.clip(ta @ tb / np.linalg.norm(ta) / np.linalg.norm(tb), -1, 1))))
    return out


def test_symmetric_circle_is_arcs_meeting_tangent():
    c = Point(50, 50).buffer(20, 256)
    segs, rep, poly = outline.build(ring_of(c), M, axis=50.0)
    assert all(s.kind == "arc" for s in segs)
    assert max(tangent_breaks(segs)) < 0.5
    assert poly.symmetric_difference(c).area < 0.02 * c.length


def test_rounded_rectangle_keeps_straight_sides():
    g = box(10, 10, 60, 40).buffer(5, quad_segs=32)
    segs, _, poly = outline.build(ring_of(g), M, axis=35.0)
    lines = [s for s in segs if s.kind == "line"]
    assert sum(np.linalg.norm(s.b - s.a) for s in lines) > 0.8 * (2 * 40 + 2 * 20)
    assert max(tangent_breaks(segs)) < 1.0          # line-arc joins stay tangent
    assert poly.symmetric_difference(g).area < 0.05 * g.length


def test_result_is_exactly_mirror_symmetric_without_axis_slit():
    g = Point(30, 30).buffer(12, 128).union(box(18, 30, 42, 50))
    _, _, poly = outline.build(ring_of(g), M, axis=30.0)
    m = affinity.scale(poly, -1, 1, origin=(30, 0))
    assert poly.symmetric_difference(m).area < 1e-3
    assert poly.geom_type == "Polygon" and len(poly.interiors) == 0
    on_axis = np.abs(np.array(poly.exterior.coords)[:, 0] - 30) < 1e-3
    assert on_axis.sum() <= 4                          # no slit running along the axis


def test_short_drawn_steps_keep_sharp_corners_in_fine_zone():
    # a leg ending in a boot: 3 px ledge then a 2 px step, as drawn at the ankle
    g = Polygon([(0, 0), (10, 0), (10, 20), (13, 20), (13, 24), (11, 24), (11, 30), (0, 30)])
    coarse, _, pc = outline.build(ring_of(g), M)
    fine, _, pf = outline.build(ring_of(g), M, fine_zone=(18, 31))
    assert pf.symmetric_difference(g).area < pc.symmetric_difference(g).area
    corners = [b for b in tangent_breaks(fine) if b > 45]
    assert len(corners) >= 7                           # every step corner survives


def test_forced_corner_lands_on_given_vertex():
    g = Polygon([(0, 0), (20, 0), (20, 10), (10, 13), (0, 10)])   # shallow 33 deg apex
    segs, _, _ = outline.build(ring_of(g, noise=0.0), M, corners_at=[(10, 13)])
    assert min(np.linalg.norm(s.a - [10, 13]) for s in segs) < 1e-6


def test_symmetric_part_mirrors_side_pieces_and_drops_left_copies():
    right = Point(40, 10).buffer(3, 64); left = affinity.scale(right, -1, 1, origin=(30, 0))
    left = affinity.translate(left, -0.4, 0.3)          # drawn slightly off
    poly, rings, _ = outline.symmetric_part(right.union(left), M, 30.0)
    assert len(getattr(poly, "geoms", [poly])) == 2
    assert poly.symmetric_difference(affinity.scale(poly, -1, 1, origin=(30, 0))).area < 1e-3


def test_arc_stroke_recovers_radius_and_width():
    U = 6; H, W = 40 * U, 60 * U
    gy, gx = np.mgrid[0:H, 0:W]; X = (gx + .5) / U; Y = (gy + .5) / U
    d = np.hypot(X - 30, Y - 50)
    mask = (np.abs(d - 30) < 1.0) & (Y < 26) & (np.abs(X - 30) < 12)
    poly, segs, info = strokes.arc_stroke(mask, X, Y, U)
    _, ls, li = strokes.line_stroke((np.abs(Y - 10) < 1) & (np.abs(X - 30) < 10), X, Y, U)
    assert abs(li['width'] - 2.0) < 0.2 and abs(li['length'] - 20) < 0.3 and len(ls) == 4
    assert abs(info["R"] - 30) < 1.5 and abs(info["width"] - 2.0) < 0.3
    assert len(segs) == 4 and all(s.kind == "arc" for s in segs)


def test_coverage_crossing_finds_subpixel_edge():
    img = np.zeros((10, 20, 3), float) + 250
    img[:, :8] = 40; img[:, 8] = 40 + 0.3 * 210        # true edge at x = 8.7 (70 % dark in pixel 8)
    cov = pixels.coverage(img, (40, 40, 40), (250, 250, 250))
    x = pixels.row_crossing(cov, 5, 2, 15)
    assert abs(x - 8.7) < 0.15


def test_changed_outside_band_detects_only_outside_moves():
    a = box(0, 0, 10, 30); b = box(0, 0, 10, 30).union(box(10, 25, 11, 30))
    assert checks.changed_outside(a, b, (24, 31)) < 1e-9
    assert checks.changed_outside(a, b, (0, 20)) > 0.5


def test_mirror_offset_reports_drawn_asymmetry():
    U = 6; m = np.zeros((40 * U, 60 * U), bool)
    m[10 * U:12 * U, 35 * U:45 * U] = True                     # right brow
    m[11 * U + 3:13 * U + 3, 15 * U:25 * U] = True              # left brow 1.5 px lower
    dx, dy = checks.mirror_offset(m, 30.0, U)
    assert abs(dy - 1.5) < 0.3 and abs(dx) < 0.3
