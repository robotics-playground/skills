import numpy as np, cv2, ezdxf
from shapely.geometry import box, Point, Polygon
from shapely.ops import unary_union
from figfront import unmix, calibrate, contours, geom, layers, export

PAL = {'bg': (251, 254, 219), 'blue': (212, 159, 40), 'black': (45, 45, 45)}


def aa_square(n=80, s=8):
    """black square on light background, anti-aliased by box downsampling"""
    big = np.zeros((n * s, n * s, 3), np.float32); big[:] = PAL['bg']
    big[20 * s + 3:60 * s + 3, 20 * s + 3:60 * s + 3] = PAL['black']
    return big.reshape(n, s, n, s, 3).mean((1, 3)).astype(np.uint8)


def test_unmix_no_false_rim():
    img = aa_square()
    lab, names = unmix.classify(img, PAL, blur_px=0.3)
    assert (lab == names.index('blue')).sum() == 0          # the black|bg blend must never become 'blue'
    black = lab == names.index('black')
    assert abs(black.sum() - 40 * 40) < 80


def test_simulated_widths_monotonic():
    w = calibrate.simulated_widths((1.0, 2.0, 3.0), trials=40)
    assert w[1.0] < w[2.0] < w[3.0]


def test_metrics_budget():
    m = calibrate.metrics(0.3, 1.6, 0.03, sim={1.0: 1.4, 1.5: 1.7, 2.0: 2.2})
    assert 1.2 < m['effective_source_pixel_px'] < 1.5
    assert abs(m['max_shift_px'] - 0.25 * m['effective_source_pixel_px']) < 1e-6


def _noisy(poly_pts, amp=0.03, seed=0):   # ~ measured edge noise after colour unmixing
    r = contours.resample(np.vstack([poly_pts, poly_pts[:1]]), 0.25)[:-1]
    return r + np.random.default_rng(seed).normal(0, amp, r.shape)


M = dict(effective_source_pixel_px=1.33, smoothing_sigma_px=0.17, max_shift_px=0.33)


def test_square_keeps_sharp_corners():
    sq = np.array([[0, 0], [30, 0], [30, 30], [0, 30]], float)
    out, nc = contours.smooth_ring(_noisy(sq), M)
    assert nc == 4
    d = [min(np.hypot(*(out - c).T)) for c in sq]
    assert max(d) < 0.6


def test_round_tip_not_spiked():
    t = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    circ = np.c_[10 * np.cos(t), 10 * np.sin(t)]
    out, nc = contours.smooth_ring(_noisy(circ), M)
    assert nc == 0
    r = np.hypot(out[:, 0], out[:, 1]); assert r.max() - r.min() < 0.8


def test_fit_circle_exact():
    t = np.linspace(0.3, 2.5, 50); pts = np.c_[5 + 20 * np.cos(t), -3 + 20 * np.sin(t)]
    cx, cy, R, rms = geom.fit_circle(pts); assert abs(cx - 5) < 1e-6 and abs(R - 20) < 1e-6 and rms < 1e-6


def test_join_circles_equal_width():
    c1, c2 = (0, 10, 30), (0, 0, 40); y = geom.join_circles_y(c1, c2)
    assert abs((30 ** 2 - (y - 10) ** 2) - (40 ** 2 - (y - 0) ** 2)) < 1e-6


def test_straight_v_shape():
    v = geom.straight_v(0, 0, 2, 5, 3, 15, 20)
    assert v.is_valid and abs(v.bounds[3] - 20) < 1e-9


def test_visible_regions_tile():
    parts = {'back': box(0, 0, 10, 10), 'mid': box(5, 0, 15, 10), 'front': Point(7, 5).buffer(3)}
    vis = layers.visible_regions(parts, ['back', 'mid', 'front'])
    u = unary_union(list(vis.values())); assert abs(u.area - unary_union(list(parts.values())).area) < 1e-6
    assert vis['back'].intersection(vis['mid']).area < 1e-9 and vis['mid'].intersection(vis['front']).area < 1e-9


def test_dxf_splines(tmp_path):
    g = box(0, 0, 10, 5).union(Point(10, 2.5).buffer(2.5))
    p = tmp_path / 'x.dxf'; export.write_dxf({'PART': g}, str(p))
    types = {e.dxftype() for e in ezdxf.readfile(str(p)).modelspace()}
    assert 'SPLINE' in types and 'LINE' in types
