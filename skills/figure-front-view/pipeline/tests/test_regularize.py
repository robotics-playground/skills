import numpy as np
from shapely.geometry import Polygon, box, Point
from figfront import regularize, checks, contours

M = dict(effective_source_pixel_px=1.33, smoothing_sigma_px=0.17, max_shift_px=0.33)
rng = np.random.default_rng(1)


def jitter(p, a=0.15):            # drawing jitter well below 0.5 effective px
    return p + rng.normal(0, a / 3, p.shape)


def test_straight_run_becomes_line():
    seg = jitter(np.c_[np.linspace(0, 40, 160), 0.02 * np.linspace(0, 40, 160)])
    e = regularize.regularize_run(seg, 0.5 * 1.33)
    assert e.kind == 'line'


def test_circular_run_becomes_arc_with_right_radius():
    t = np.linspace(0.4, 2.6, 200); seg = jitter(np.c_[30 * np.cos(t), 30 * np.sin(t)])
    e = regularize.regularize_run(seg, 0.5 * 1.33)
    assert e.kind == 'arc' and abs(e.params['radius'] - 30) < 0.3


def test_wobbly_curve_becomes_one_smooth_spline():
    x = np.linspace(0, 60, 400); y = 8 * np.sin(x / 25) + 0.25 * np.sin(x * 1.3)   # big curve + small wobble
    e = regularize.regularize_run(np.c_[x, y], 0.5 * 1.33)
    assert e.kind in ('spline', 'arc')
    # the wobble is removed: curvature sign changes far fewer than in the input
    def sign_changes(p):
        d1 = np.gradient(p, axis=0); d2 = np.gradient(d1, axis=0); k = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
        return int((np.diff(np.sign(k[5:-5])) != 0).sum())
    assert sign_changes(e.pts) < sign_changes(np.c_[x, y]) / 3


def test_real_small_feature_survives():
    x = np.linspace(0, 40, 300); y = np.where(np.abs(x - 20) < 4, -1.2 * np.cos((x - 20) / 4 * np.pi / 2) ** 2, 0)  # 1.2 px bulge
    e = regularize.regularize_run(np.c_[x, y], 0.5 * 1.33)
    assert e.kind == 'spline' and e.pts[:, 1].min() < -0.8


def test_square_with_round_corner_polygon():
    poly = box(0, 0, 30, 20).union(Point(30, 10).buffer(10))     # straight sides + a semicircle end
    reg, rep = regularize.regularize_polygon(poly, M)
    kinds = sorted(e.kind for _, e in rep)
    assert 'line' in kinds and ('arc' in kinds or 'pieces' in kinds)
    assert abs(reg.area - poly.area) / poly.area < 0.01


def test_scope_guard_flags_untouched_change():
    prev = {'FACE': box(0, 0, 10, 10), 'CHIN': box(0, 10, 10, 12)}
    new = {'FACE': box(0, 0, 10, 11), 'CHIN': box(0, 10, 10, 13)}
    assert set(checks.changed_parts(prev, new, allowed={'CHIN'})) == {'FACE'}


def test_overlap_report_flags_back_part_stopping_short():
    helmet = box(0, 0, 20, 10)
    face_short = box(5, 9.8, 15, 20)      # barely under the helmet
    face_ok = box(5, 5, 15, 20)
    assert checks.overlap_report({'FACE': face_short, 'HELMET': helmet}, ['FACE', 'HELMET'], 1.0)
    assert not checks.overlap_report({'FACE': face_ok, 'HELMET': helmet}, ['FACE', 'HELMET'], 1.0)


def test_same_colour_neighbours():
    parts = {'JACKET': box(0, 0, 10, 10), 'PANTS': box(0, 10, 10, 20), 'FACE': box(20, 0, 30, 10)}
    col = {'JACKET': 'black', 'PANTS': 'black', 'FACE': 'white'}
    assert checks.same_colour_neighbours(parts, col, 0.01) == [('JACKET', 'PANTS', 'black')]


def test_review_sheet_handles_all_edge_kinds(tmp_path):
    import json, cv2
    from figfront import review
    poly = box(0, 0, 30, 20).union(Point(30, 10).buffer(10))
    reg, rep = regularize.regularize_polygon(poly, M)
    bg = np.full((60, 60, 3), 230, np.uint8)
    rows = review.review_sheet(bg, {'P': reg}, {'P': rep}, origin=(-10, -10), scale=4, out_png=str(tmp_path / 's.png'))
    json.dumps(rows)
    assert rows and (tmp_path / 's.png').exists()


def test_drop_specks_removes_islands_and_pinholes_keeps_small_parts():
    from figfront import contours
    from shapely.geometry import MultiPolygon
    body = box(0, 0, 20, 20).difference(Point(10, 10).buffer(0.2))          # pinhole
    noisy = MultiPolygon([body] + [Point(30 + i, 30).buffer(0.25) for i in range(0, 20, 2)])
    assert checks.specks({'SKIN': noisy}, 0.3)['SKIN']['islands'] == 10
    clean = contours.drop_specks(noisy, 0.3)
    assert clean.geom_type == 'Polygon' and len(clean.interiors) == 0
    assert not checks.specks({'SKIN': clean}, 0.3)
    nose = Point(0, 0).buffer(0.25)                                           # a real tiny part
    assert not contours.drop_specks(nose, 0.3).is_empty
