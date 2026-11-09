"""Pixel calibration, part geometry (exact where the print is geometric, traced + regularized elsewhere),
mirror symmetry, hidden-part completion. Output: work/parts_px.pkl (shapely, artwork px, y down)."""
import json
import pickle
import cv2
import numpy as np
from scipy import ndimage as ndi
from scipy.optimize import minimize
from shapely.geometry import Polygon, box, Point, MultiPolygon
from shapely import affinity
from shapely.ops import unary_union
from figfront import calibrate, contours, regularize, geom
from figfront.contours import disk_op
import config as C

U = C.UP
F = json.load(open(C.WORK / "fits.json"))
Mk = dict(np.load(C.WORK / "masks.npz"))
AX, T_W, sole_y = F["AX"], F["T_W"], F["sole_y"]
Hg, Wg = Mk["fg"].shape
gy, gx = np.mgrid[0:Hg, 0:Wg]; X = (gx + 0.5) / U; Y = (gy + 0.5) / U
art = cv2.imread(str(C.ARTWORK))
ORIGIN = (0.5 / U, 0.5 / U)     # mask_rings returns j/UP; pixel-edge convention needs (j + 0.5)/UP


def rings_of(mask, sigma=1.4):
    return contours.mask_rings(mask, U, ORIGIN, sigma=sigma)


def raster(g):
    m = np.zeros((Hg, Wg), np.uint8)
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        cv2.fillPoly(m, [(np.array(p.exterior.coords) * U - 0.5).round().astype(np.int32)], 1)
        for h in p.interiors: cv2.fillPoly(m, [(np.array(h.coords) * U - 0.5).round().astype(np.int32)], 0)
    return m.astype(bool)


# ---------------- 1. pixel calibration (raw, unsymmetrised artwork) ----------------
lab = np.load(C.WORK / "labels.npz")["lab"]; names = list(np.load(C.WORK / "labels.npz")["names"])
blue_raw = lab == names.index("blue")
fg_raw = (lab != names.index("bg")) & (Y < sole_y)
hr = max(rings_of(blue_raw), key=len); hr = hr[hr[:, 1] < 40]                    # helmet dome | background
lr = max(rings_of(fg_raw), key=len); lr = lr[(lr[:, 1] > 200) & (lr[:, 1] < 260)]  # leg outer edges | background
P = C.PALETTE
to_c = lambda r: r - 0.5                                                          # pixel-centre coords for sampling
w1 = calibrate.edge_widths(art, to_c(hr), P["blue"], P["bg"])
w2 = calibrate.edge_widths(art, to_c(lr), P["black"], P["bg"])
meas = float(np.median(np.r_[w1, w2]))
sim = calibrate.simulated_widths()
eff_guess = float(np.interp(meas, list(sim.values()), list(sim)))
noise = float(np.sqrt(np.mean([calibrate.edge_noise(hr, 1.5 * eff_guess) ** 2, calibrate.edge_noise(lr, 1.5 * eff_guess) ** 2])))
plate_h_px = sole_y - Y[Mk["HELMET"]].min() + 0.5 / U
mm_px = C.FIGURE_HEIGHT_MM / plate_h_px
m = calibrate.metrics(mm_px, meas, noise, sim)
m.update(edge_samples=[len(w1), len(w2)], figure_height_px=float(plate_h_px))
json.dump(m, open(C.OUT / "pixel_metrics.json", "w"), indent=1)
print(json.dumps({k: m[k] for k in ("nominal_mm_per_px", "measured_edge_width_px", "effective_source_pixel_px",
                                    "effective_source_pixel_mm", "edge_noise_rms_px", "max_shift_px", "max_shift_mm")}))
EFF = m["effective_source_pixel_px"]
SPECK = max(0.3 / mm_px ** 2, (2 * EFF) ** 2)


# ---------------- 2. helpers: trace -> outline (lines + tangent arcs, corners kept) -> exact symmetry ----------------
from figfront import outline as OL


def trace(mask, small=False):
    g = contours.mask_to_polygon(mask, m, U, ORIGIN, small=small)
    return contours.drop_specks(g, SPECK if not small else 0.5)


SEGS = {}


def clean(name, g, **kw):
    """Rebuild a symmetric part from line/arc segments: pieces crossing the axis are fitted on the right
    half and mirrored; pieces wholly on the right are fitted and copied mirrored; left pieces are dropped."""
    pieces = [p for p in getattr(g, "geoms", [g]) if not p.is_empty]
    out, rings, rep = [], [], []
    for p in pieces:
        x0, _, x1, _ = p.bounds
        if x1 <= AX: continue
        if x0 < AX < x1:
            poly, rg, r = OL.build_geom(p, m, AX, **kw); out.append(poly); rings += rg; rep += r
        else:
            poly, rg, r = OL.build_geom(p, m, None, **kw); rep += r
            out += [poly, affinity.scale(poly, -1, 1, origin=(AX, 0))]
            rings += rg + [([s.mirrored(AX) for s in reversed(so)], [[s.mirrored(AX) for s in reversed(h)] for h in hs]) for so, hs in rg]
    # weld halves and drop zero-area slits left along the axis
    g = unary_union([q.buffer(1e-3) for q in out]).buffer(-1e-3)
    g = unary_union([Polygon(q.exterior, [h for h in q.interiors if Polygon(h).area > 0.5]) for q in getattr(g, "geoms", [g])])
    rings = [(so, [h for h in hs if OL.polygon(h).area > 0.5]) for so, hs in rings]
    SEGS[name] = rings
    return g, [r for r in rep if not (r[0] == "hole")] if all(len(hs) == 0 for _, hs in rings) else rep


def mirror_sym(g):
    big = 1e4
    right = g.intersection(box(AX, -big, big, big))
    return unary_union([right, affinity.scale(right, -1, 1, origin=(AX, 0))]).buffer(0)


def ellipse(cy, rx, ry, cx=None, n=720):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return Polygon(np.c_[(AX if cx is None else cx) + rx * np.cos(t), cy + ry * np.sin(t)])


parts, edges = {}, {}
o, i = F["outer"], F["inner"]


# ---------------- 4. HELMET: traced, regularized, symmetric ----------------
parts["HELMET"], edges["HELMET"] = clean("HELMET", trace(Mk["HELMET"]))

# ---------------- 5. FACE: visible = helmet opening + chin circle; hidden = full head under the helmet ----------------
helmet_px = raster(parts["HELMET"])
flap_tip_y = max(Y[Mk["HELMET"] & (np.abs(X - AX) > 25) & (np.abs(X - AX) < 50)])
fv = Mk["FACE_VIS"]
# chin edge: lowest face pixel per column between the flap tips, shirt columns excluded (white neighbour)
cw = Mk["CRAVAT_VIS"]; shirt_hw = (X[cw & (Y < 115)].max() - X[cw & (Y < 115)].min()) / 2
pts = []
for c in range(Wg):
    col = np.nonzero(fv[:, c])[0]
    if len(col) == 0: continue
    x = (c + 0.5) / U; y = (col.max() + 1) / U
    if abs(x - AX) < shirt_hw + 1.5: continue
    if y > flap_tip_y - 6 and abs(x - AX) < 40: pts.append((x, y))
pts = np.array(pts)
pts = np.vstack([pts, np.c_[2 * AX - pts[:, 0], pts[:, 1]]])
cx, cy, R, rms = geom.fit_circle(pts)
print(f"chin circle cx={cx:.2f} (axis {AX:.2f}) cy={cy:.2f} R={R:.2f}px rms={rms:.3f}px, n={len(pts)}")
chin = Point(AX, cy).buffer(R, 256)
face_traced = trace(fv)
face_vis = unary_union([face_traced.intersection(box(-1e4, -1e4, 1e4, flap_tip_y - 4)),
                        chin.intersection(box(-1e4, flap_tip_y - 6, 1e4, 1e4))]).buffer(0)
face_vis = mirror_sym(face_vis.buffer(0.3).buffer(-0.3))
# hidden head: superellipse reaching well under the helmet (>= 2 mm), kept inside the helmet's outer
# outline (shrunk 1 mm) so it never shows; the band along the helmet's inner edge stays covered
tuck = 2.0 / mm_px
fx0, fy0, fx1, fy1 = face_vis.bounds
a_ = (fx1 - fx0) / 2 + tuck; top = fy0 - tuck; cy_h = (fy0 + fy1) / 2
t = np.linspace(0, 2 * np.pi, 720, endpoint=False); n_ = 2.6
head = Polygon(np.c_[AX + a_ * np.sign(np.cos(t)) * np.abs(np.cos(t)) ** (2 / n_),
                     cy_h + (cy_h - top) * np.sign(np.sin(t)) * np.abs(np.sin(t)) ** (2 / n_)])
helmet_outer = parts["HELMET"].convex_hull           # dome + flaps; also covers the open face notch
hidden = head.intersection(helmet_outer.buffer(-1.0 / mm_px)).intersection(box(-1e4, -1e4, 1e4, flap_tip_y - 3))
parts["FACE"], edges["FACE"] = clean("FACE", unary_union([face_vis, hidden]).buffer(0.5).buffer(-0.5))

# ---------------- 6. CRAVAT: straight-line V fitted to the visible shirt ----------------
cv_vis = Mk["CRAVAT_VIS"] & ~raster(chin)
win = (Y > cy + R - 1) & (Y < Y[cv_vis].max() + 2) & (np.abs(X - AX) < 14)
y_ref = cy + R + 1.0
def vpoly(q, top_y):
    w_ref, w_v, y_v, y_tip = q
    return geom.straight_v(AX, top_y, w_ref, y_ref, w_v, y_v, y_tip)
def vloss(q):
    if q[0] <= 1 or q[1] <= 1 or q[2] <= y_ref + 2 or q[3] <= q[2] + 1: return 2.0
    a = raster(vpoly(q, y_ref - 6)) & win; b = cv_vis & win
    return 1 - (a & b).sum() / max((a | b).sum(), 1)
q0 = np.array([shirt_hw, shirt_hw + 0.8, Y[cv_vis].max() - 9, Y[cv_vis].max()])
best = min((minimize(vloss, q0 + d, method="Nelder-Mead",
                     options=dict(maxiter=900, xatol=0.01, fatol=1e-7,
                                  initial_simplex=q0 + d + np.vstack([np.zeros(4), np.eye(4) * [0.6, 0.6, 1.5, 1.0]])))
            for d in (np.zeros(4), np.array([0.5, 0, 0, 0]), np.array([-0.5, 0.5, -1, 0]))), key=lambda r: r.fun)
CQ = best.x
NOSE_Y = Y[Mk["NOSE"]].mean()
parts["CRAVAT"] = vpoly(CQ, NOSE_Y - 2.0)          # 2 px above the jacket top: the V-neck opens through the top edge
print(f"cravat V IoU={1 - best.fun:.3f}  half-width {CQ[0]:.2f}->{CQ[1]:.2f}px, V start y={CQ[2]:.2f}, tip y={CQ[3]:.2f}")

# ---------------- 7. HANDS: traced, regularized, symmetric ----------------
parts["HANDS"], edges["HANDS"] = clean("HANDS", trace(Mk["HANDS_VIS"]))

# hands bottom: two fists touching -> one arc per fist, meeting at a sharp notch on the axis.
# Each arc passes through the hand corner (bottom of edge 12) and the measured notch point, and is the
# best circle between them for the bottom edge measured column by column (left fist mirrored).
_wt = np.array(C.PALETTE["grey"], float); _bk = np.array(C.PALETTE["black"], float); _v = _wt - _bk
_W = np.clip(((art.astype(float) - _bk) @ _v) / (_v @ _v), 0, 1)
def _bottom(col):
    for y in range(150, 175):
        a0, a1 = _W[y, col], _W[y + 1, col]
        if (a0 - .5) * (a1 - .5) <= 0 and a0 != a1 and a0 > a1: return y + .5 + (.5 - a0) / (a1 - a0)
    return np.nan
hchain = SEGS["HANDS"][0][0]; hchain = hchain[:len(hchain) // 2]
k_c = next(j for j, sg in enumerate(hchain) if abs(sg.b[0] - hchain[0].a[0]) > 1 and sg.kind == "line")   # first straight side
corner = hchain[k_c].a.copy()                                   # bottom of the hand side (edge 12)
dxs = np.arange(0.5, corner[0] - AX - 0.5, 1.0)
prof = np.array([[AX + d, np.nanmean([_bottom(int(np.floor(AX + d))), _bottom(int(np.floor(AX - d)))])] for d in dxs])
notch = np.array([AX, float(np.mean(prof[:2, 1]))])
mid = (notch + corner) / 2; nrm = np.array([-(corner - notch)[1], (corner - notch)[0]]); nrm /= np.linalg.norm(nrm)
from scipy.optimize import minimize_scalar as _ms
def _cost(t):
    c = mid + t * nrm; R_ = np.linalg.norm(corner - c)
    return np.mean((np.hypot(*(prof[2:] - c).T) - R_) ** 2)
t_best = _ms(_cost, bounds=(-400, 400), method="bounded").x
fc = mid + t_best * nrm; fR = float(np.linalg.norm(corner - fc))
fist = OL._single_arc(np.vstack([notch, prof[len(prof) // 2], corner]), None, None)
fist = OL.Seg("arc", notch.copy(), corner.copy(), fc, fR, fist.sweep)
fist_dev = float(np.max(np.abs(np.hypot(*(prof[2:] - fc).T) - fR)))
hchain = [fist] + hchain[k_c:]
half = OL.polygon(hchain, open_chain=True)
parts["HANDS"] = unary_union([half.buffer(0.02), Polygon([(2 * AX - x, y) for x, y in half.exterior.coords]).buffer(0.02)]).buffer(-0.02)
SEGS["HANDS"] = [(hchain + [sg.mirrored(AX) for sg in reversed(hchain)], [])]
edges["HANDS"] = [("outer", "run", "arc", 1, fist_dev, [fist])] + [r for r in edges["HANDS"] if not (r[5] and abs(r[5][0].a[0] - AX) < 1e-6 and r[5][0].a[1] > corner[1] - 1)]
FIST = dict(radius=fR, centre=fc.tolist(), notch=notch.tolist(), corner=corner.tolist(), max_dev=fist_dev,
            notch_angle_deg=float(180 - 2 * np.degrees(np.arctan2(abs(fc[0] - AX), abs(fc[1] - notch[1])))))
print(f"fist arc R={fR:.2f}px through notch {notch.round(2)} and corner {corner.round(2)}, max dev {fist_dev:.2f}px")

# ---------------- 3. ARMS: oval ring that ends on the hands' side edges (edge 12) ----------------
# Where the ring edge is visible (outside the jacket, |x - axis| >= jacket half-width) it is the free-fit
# ellipse. Inside the jacket width the artwork draws black on black, so it holds no evidence there: each
# edge leaves its ellipse tangentially and runs as a biarc to the end of the straight hand side, meeting
# it at the ellipse's own angle. Arms + hands then form one closed loop with no step.
hs = [sg for so, _ in SEGS["HANDS"] for sg in so if sg.kind == "line" and abs(sg.a[0] - sg.b[0]) < 1e-6 and sg.a[0] > AX + 5]
side = max(hs, key=lambda sg: abs(sg.a[1] - sg.b[1]))
HX = float(side.a[0]); H_TOP, H_BOT = sorted([float(side.a[1]), float(side.b[1])])
DX = HX - AX


def ell_pt(e, dx):          # lower-half point and unit tangent (direction of decreasing x) at x = AX + dx
    phi = np.arccos(dx / e["rx"])
    P = np.array([AX + e["rx"] * np.cos(phi), e["cy"] + e["ry"] * np.sin(phi)])
    T = np.array([-e["rx"] * np.sin(phi), e["ry"] * np.cos(phi)]); return P, T / np.linalg.norm(T), phi


def blend(e, end_y):
    P0, T0, phi0 = ell_pt(e, T_W); _, T1, _ = ell_pt(e, DX)
    P1 = np.array([HX, end_y])
    b2 = OL.biarc(P0, T0, P1, T1)
    segs = [b2[0], OL._flip(b2[1])]
    # how far the blend leaves the ellipse it replaces (hidden zone)
    q = np.vstack([sg.points(0.1) for sg in segs])
    u = (q[:, 0] - AX) / e["rx"]; v = (q[:, 1] - e["cy"]) / e["ry"]
    return segs, phi0, float(np.max(np.abs(np.hypot(u, v) - 1) * min(e["rx"], e["ry"])))


bo, phi_o, dev_o = blend(o, H_BOT)
bi, phi_i, dev_i = blend(i, H_TOP)
print(f"arm ends: outer blend leaves the ellipse by <= {dev_o:.2f}px, inner by <= {dev_i:.2f}px (hidden zone)")


def ell_arc(e, phi_a, phi_b, n=1400):        # phi from phi_a down through the top (phi = -90 deg) to phi_b
    t = np.linspace(phi_a, phi_b, n)
    return np.c_[AX + e["rx"] * np.cos(t), e["cy"] + e["ry"] * np.sin(t)]


mir = lambda segs: [sg.mirrored(AX) for sg in reversed(segs)]
pts = lambda segs: np.vstack([sg.points(0.1) for sg in segs])
# loop: right hand corner (bottom) -> outer blend -> outer ellipse over the top -> left outer blend ->
# left hand side -> left inner blend -> inner ellipse back over the top -> right inner blend -> right hand side
loop = np.vstack([pts(bo)[::-1],                              # right hand corner (bottom) -> outer ellipse
                  ell_arc(o, phi_o, -np.pi - phi_o),            # outer ellipse over the top to the left
                  pts(mir(bo))[::-1],                           # -> left hand corner (bottom); side edge up
                  pts(mir(bi)),                                 # left hand corner (top) -> inner ellipse
                  ell_arc(i, -np.pi - phi_i, phi_i),            # inner ellipse over the top to the right
                  pts(bi)])                                     # -> right hand corner (top); side edge down
parts["ARMS"] = Polygon(loop).buffer(0)
SEGS["ARMS"] = [(bo + mir(bo) + bi + mir(bi) + [OL.Seg("line", np.array([HX, H_TOP]), np.array([HX, H_BOT])),
                 OL.Seg("line", np.array([2 * AX - HX, H_BOT]), np.array([2 * AX - HX, H_TOP]))], [])]
ARM_ELLIPSE_ARCS = [dict(e=o, phi_from=float(phi_o), phi_to=float(np.pi - phi_o)), dict(e=i, phi_from=float(phi_i), phi_to=float(np.pi - phi_i))]

# ---------------- 8. BODY (jacket + pants + boots, one plate) ----------------
# jacket = straight-sided panel from nose level down (exposed when the ring turns, video t03/t12);
# below the ring the outline is traced (hips, legs, crotch gap, boots).
jacket = box(AX - T_W, NOSE_Y, AX + T_W, 185.0)
ring_px = raster(parts["ARMS"])
lower = Mk["BLACK"] & ~ring_px & (Y > o["cy"] + 20)
# the 1 px opening cleans partition slivers; in the boot zone it would round the drawn ankle steps
lower = np.where(Y < sole_y - 32.0, disk_op(lower, "open", 1.0 * U), lower)
body_mask = raster(jacket) | lower
body_mask = ndi.binary_fill_holes(body_mask) & (Y < sole_y)
BOOT_ZONE = (sole_y - 32.0, sole_y - 0.5)          # ankle steps and boot toe
built, _ = clean("BODY", trace(body_mask), fine_zone=BOOT_ZONE)
# above the waist the plate is the exact jacket panel; the built outline takes over a little above the
# waist corner, where both run along the same straight jacket side
wr = (lower.sum(1) / U) / 2
yy = (np.arange(Hg) + 0.5) / U
Y_WAIST = float(yy[np.argmax((yy > o["cy"] + o["ry"]) & (wr > T_W + 1.0))])
y_s = Y_WAIST - 3.0
body = unary_union([built.intersection(box(-1e4, y_s, 1e4, 1e4)), jacket.intersection(box(-1e4, -1e4, 1e4, y_s + 1e-3))])
body = body.buffer(1e-4).buffer(-1e-4).difference(parts["CRAVAT"]).buffer(0)
# second pass on exact geometry (segments for CAD): its vertices are real, so a low corner angle is safe
vtx = [c for c in list(parts["CRAVAT"].exterior.coords)[:-1] + list(jacket.exterior.coords)[:-1]
       + list(body.exterior.coords)[:-1] if body.exterior.distance(Point(c)) < 1e-6 and
       (Point(c).distance(parts["CRAVAT"].exterior) < 1e-6 or Point(c).distance(jacket.exterior) < 1e-6)]
vtx = [v for v in vtx if v[1] < y_s]
parts["BODY"], edges["BODY"] = clean("BODY", body, corners_at=vtx, fine_zone=BOOT_ZONE)

# Round-2 scope: only the boots were asked for. Keep the round-1 outline down to a segment joint just
# above the boot zone and bridge tangentially into the new boot segments.
R1_PATH = C.WORK / "parts_px_round1.pkl"
if R1_PATH.exists():
    def right_chain(rings):
        so = rings[0][0]; ch = so[:len(so) // 2]
        return ch if ch[0].a[1] < ch[-1].b[1] else [x for x in reversed([sg.mirrored(AX).mirrored(AX) for sg in ch])]

    def tan_end(sg, at_end):
        q = sg.points(0.01); v = (q[-1] - q[-2]) if at_end else (q[1] - q[0]); return v / np.linalg.norm(v)

    R1 = pickle.load(open(R1_PATH, "rb"))["segs"]["BODY"][0][0]; R1 = R1[:len(R1) // 2]
    NB = SEGS["BODY"][0][0]; NB = NB[:len(NB) // 2]
    Y_C = BOOT_ZONE[0] - 6.0
    assert np.linalg.norm(R1[0].a - NB[0].a) < 1e-3, "chains start at different points"

    def split_at_y(sg, y):
        """point where segment sg crosses y, and the two halves"""
        q = sg.points(0.005); k = int(np.argmin(np.abs(q[:, 1] - y))); P = q[k]
        if sg.kind == "line": return P, OL.Seg("line", sg.a, P), OL.Seg("line", P, sg.b)
        ang = lambda p: np.arctan2(p[1] - sg.c[1], p[0] - sg.c[0])
        sw1 = (ang(P) - ang(sg.a) + np.pi) % (2 * np.pi) - np.pi
        if np.sign(sw1) != np.sign(sg.sweep): sw1 += np.sign(sg.sweep) * 2 * np.pi
        return P, OL.Seg("arc", sg.a, P, sg.c, sg.r, sw1), OL.Seg("arc", P, sg.b, sg.c, sg.r, sg.sweep - sw1)

    def is_corner(ch, j):          # joint between ch[j-1] and ch[j]
        return np.degrees(np.arccos(np.clip(tan_end(ch[j - 1], True) @ tan_end(ch[j], False), -1, 1))) > 20

    jo = next(j for j, sg in enumerate(R1) if sg.b[1] > Y_C)               # R1 enters the zone (inner leg)
    ji = max(j for j, sg in enumerate(R1) if sg.a[1] > Y_C)                # R1 leaves the zone (outer leg)
    Po, keep_o, _ = split_at_y(R1[jo], Y_C); _, _, keep_i = split_at_y(R1[ji], Y_C); Pi = keep_i.a
    # Boots: the artwork draws square pixel steps at the ankles. Measured as 50 % black-coverage crossings
    # on the raw artwork, both boots (left one mirrored onto the right), averaged -> symmetric staircase.
    bg_ = np.array(C.PALETTE["bg"], float); bk_ = np.array(C.PALETTE["black"], float); vv = bg_ - bk_
    COV = np.clip(((bg_ - art.astype(float)) @ vv) / (vv @ vv), 0, 1)
    DL = F["AX"] - F["AX_LEGS"]

    def xcross(row, x_from, x_to):
        st = 1 if x_to > x_from else -1
        for x in range(x_from, x_to, st):
            a0, a1 = COV[row, x], COV[row, x + st]
            if (a0 - .5) * (a1 - .5) <= 0 and a0 != a1: return x + .5 + st * (.5 - a0) / (a1 - a0)
        return np.nan

    def ycross(col, y_from, y_to):
        st = 1 if y_to > y_from else -1
        for y in range(y_from, y_to, st):
            a0, a1 = COV[y, col], COV[y + st, col]
            if (a0 - .5) * (a1 - .5) <= 0 and a0 != a1: return y + .5 + st * (.5 - a0) / (a1 - a0)
        return np.nan

    def raw_col(xc, right): return int(np.floor((xc if right else 2 * AX - xc) - DL))
    def inner(row): return np.mean([xcross(row, 150, 190) + DL, 2 * AX - (xcross(row, 105, 70) + DL)])
    def outer(row): return np.mean([xcross(row, 205, 175) + DL, 2 * AX - (xcross(row, 50, 80) + DL)])
    def ledge(xc, y_from, y_to): return np.mean([ycross(raw_col(xc, r), y_from, y_to) for r in (True, False)])
    def step_y(fn, rows, x_hi, x_lo):           # y where the row crossing passes halfway between two levels
        ys = np.array(rows) + .5; xs = np.array([fn(r) for r in rows]); mid = (x_hi + x_lo) / 2
        k = int(np.argmax(xs < mid)); return float(np.interp(mid, [xs[k], xs[k - 1]], [ys[k], ys[k - 1]]))

    xi_leg = np.mean([inner(r) for r in range(266, 273)]); y_li = np.mean([ledge(x, 278, 268) for x in (166.5, 167.5)])
    xi1 = np.mean([inner(r) for r in range(275, 278)]); xi2 = np.mean([inner(r) for r in range(280, 284)])
    xi3 = np.mean([inner(r) for r in range(288, 299)])
    y_s1 = step_y(inner, range(276, 282), xi1, xi2); y_s2 = step_y(inner, range(282, 289), xi2, xi3)
    xo_leg = np.mean([outer(r) for r in range(271, 274)]); y_lo = np.mean([ledge(x, 278, 268) for x in (189.5, 190.5)])
    xo1 = np.mean([outer(r) for r in range(275, 279)]); x_toe = np.mean([outer(r) for r in range(293, 299)])
    sx = np.array([189.5, 190.5, 191.5, 192.5]); sy = np.array([ledge(x, 276, 290) for x in sx])
    ka, kb = np.polyfit(sx, sy, 1)                                  # slanted step under the outer notch
    toe = np.array([[outer(r), r + .5] for r in range(284, 293)])
    Jx = toe[0, 0]; J = np.array([Jx, ka * Jx + kb]); K = np.array([xo1, ka * xo1 + kb])
    I = np.array([x_toe, toe[-1, 1]])
    V = lambda x, y: np.array([float(x), float(y)])
    A_, B_, C_, D_ = V(xi_leg, y_li), V(xi1, y_li), V(xi1, y_s1), V(xi2, y_s1)
    E_, F_, G_, H_ = V(xi2, y_s2), V(xi3, y_s2), V(xi3, sole_y), V(x_toe, sole_y)
    L_, M_ = V(xo1, y_lo), V(xo_leg, y_lo)
    curve_pts = np.vstack([I, toe[::-1][1:], J])                    # toe front, from the vertical up to the step
    dense = OL.resample(curve_pts, 0.1)
    toe_segs = OL.fit_biarcs(dense, np.array([0.0, -1.0]), OL._tangent(dense, len(dense) - 1, 4), 0.15, w=4)
    Ln = lambda p, q: OL.Seg("line", p.copy(), q.copy())
    boot = [Ln(A_, B_), Ln(B_, C_), Ln(C_, D_), Ln(D_, E_), Ln(E_, F_), Ln(F_, G_), Ln(G_, H_), Ln(H_, I)] + toe_segs +            [Ln(J, K), Ln(K, L_), Ln(L_, M_)]
    BOOT_FIT = dict(inner_leg_x=xi_leg, inner_ledge_y=y_li, inner_x=[xi1, xi2, xi3], inner_steps_y=[y_s1, y_s2],
                    outer_leg_x=xo_leg, outer_ledge_y=y_lo, outer_notch_x=xo1, outer_step_line=[ka, kb], toe_x=x_toe)
    print("boot staircase (px):", {k_: (np.round(v_, 2).tolist() if isinstance(v_, (list, np.ndarray)) else round(float(v_), 2)) for k_, v_ in BOOT_FIT.items()})
    br1 = OL._arc_from(Po, tan_end(keep_o, True), A_)                  # tangent to the leg, corner at the ledge
    br2 = OL._flip(OL._arc_from(Pi, -tan_end(keep_i, False), M_))
    chain = R1[:jo] + [keep_o, br1] + boot + [br2, keep_i] + R1[ji + 1:]
    Ko, Ki = A_, M_
    Jo, Ji = Po, Pi
    half = OL.polygon(chain, open_chain=True)
    mirr = Polygon([(2 * AX - x, y) for x, y in half.exterior.coords])
    parts["BODY"] = unary_union([half.buffer(0.02), mirr.buffer(0.02)]).buffer(-0.02)
    SEGS["BODY"] = [(chain + [sg.mirrored(AX) for sg in reversed(chain)], [])]
    # edge report from the spliced chain: runs between joins that break more than 20 deg
    runs, cur = [], [chain[0]]
    for prev, sg in zip(chain, chain[1:]):
        if np.degrees(np.arccos(np.clip(tan_end(prev, True) @ tan_end(sg, False), -1, 1))) > 20:
            runs.append(cur); cur = []
        cur.append(sg)
    runs.append(cur)
    def kind_of(r):
        if len(r) == 1: return r[0].kind
        return "curve"
    edges["BODY"] = [("outer", "run", kind_of(r), len(r), 0.0, r) for r in runs]
    print(f"BODY: round-1 outline kept above y={Y_C:.1f}; bridges {Jo.round(2)}->{Ko.round(2)} and {Ki.round(2)}->{Ji.round(2)}")
print(f"waist corner y={Y_WAIST:.2f}")

# ---------------- 9. face inlays ----------------
# glasses and nose: traced + outline builder. Eyebrows and mouth lines: constant-width arc strokes with
# round caps (centre line = circle fitted to the stroke skeleton; width from area and length).
from skimage.morphology import skeletonize
from scipy.optimize import least_squares as _lsq


def arc_stroke(mask, on_axis):
    # centre line: mean y per grid column (strokes here are wide, not tall); a skeleton grows spurs at
    # thick stroke ends. Columns within half a stroke width of each end are caps, not centre line.
    cols = np.nonzero(mask.any(0))[0]
    P = np.array([[X[0, c], Y[mask[:, c], c].mean()] for c in cols])
    w0 = (mask.sum() / U ** 2) / max(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)), 1e-6)
    s_ = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    P = P[(s_ >= w0 / 2) & (s_ <= s_[-1] - w0 / 2)]
    if on_axis:
        def res(q): return np.hypot(P[:, 0] - AX, P[:, 1] - q[0]) - q[1]
        ymid = P[:, 1].mean(); q0 = [ymid - 60, 60]
        qa = _lsq(res, q0).x; qb = _lsq(res, [ymid + 60, 60]).x
        q = qa if np.abs(res(qa)).mean() < np.abs(res(qb)).mean() else qb
        c = np.array([AX, q[0]]); R = abs(q[1])
    else:
        cx, cy, R, _ = geom.fit_circle(P); c = np.array([cx, cy])
    ang = np.unwrap(np.sort(np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0])))
    ang = np.arctan2(P[:, 1] - c[1], P[:, 0] - c[0]); am = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())
    rel = (ang - am + np.pi) % (2 * np.pi) - np.pi
    a0, a1 = am + rel.min(), am + rel.max()
    if on_axis:
        h = 0.5 * (rel.max() - rel.min()); am = np.pi / 2 if c[1] < P[:, 1].mean() else -np.pi / 2; a0, a1 = am - h, am + h
    Ls = R * (a1 - a0); A = mask.sum() / U ** 2
    w = (-Ls + np.sqrt(Ls ** 2 + np.pi * A)) / (np.pi / 2)
    t = np.linspace(a0, a1, 200); centre = np.c_[c[0] + R * np.cos(t), c[1] + R * np.sin(t)]
    from shapely.geometry import LineString
    poly = LineString(centre).buffer(w / 2, quad_segs=32)
    e0, e1 = centre[0], centre[-1]
    so = OL.Seg("arc", c + (R + w / 2) * np.r_[np.cos(a0), np.sin(a0)], c + (R + w / 2) * np.r_[np.cos(a1), np.sin(a1)], c, R + w / 2, a1 - a0)
    cap1 = OL.Seg("arc", so.b, c + (R - w / 2) * np.r_[np.cos(a1), np.sin(a1)], e1, w / 2, np.pi)
    si = OL.Seg("arc", cap1.b, c + (R - w / 2) * np.r_[np.cos(a0), np.sin(a0)], c, R - w / 2, -(a1 - a0))
    cap0 = OL.Seg("arc", si.b, so.a, e0, w / 2, np.pi)
    segs = [so, cap1, si, cap0]
    if OL.polygon(segs).symmetric_difference(poly).area > 0.1 * poly.area:      # caps turn the other way
        cap1.sweep = cap0.sweep = -np.pi
    return OL.polygon(segs), segs, dict(centre=c.tolist(), R=float(R), width=float(w), a0_deg=float(np.degrees(a0)),
                                        a1_deg=float(np.degrees(a1)), skeleton_rms=float(np.std(np.hypot(*(P - c).T) - R)))


def line_stroke(mask):
    """straight constant-width stroke with round ends: centre line fitted to per-column centroids"""
    cols = np.nonzero(mask.any(0))[0]
    P = np.array([[X[0, c], Y[mask[:, c], c].mean()] for c in cols])
    c0 = P.mean(0); d = np.linalg.svd(P - c0)[2][0]; d = d if d[0] > 0 else -d
    t = (P - c0) @ d; L = t.max() - t.min()
    A = mask.sum() / U ** 2
    q_ = 1 - np.pi / 4                                   # area = L*w - q_*w^2 (L includes both round ends)
    w = (L - np.sqrt(L ** 2 - 4 * q_ * A)) / (2 * q_)
    e0 = c0 + (t.min() + w / 2) * d; e1 = c0 + (t.max() - w / 2) * d
    n = np.array([-d[1], d[0]]) * w / 2
    a0, a1 = np.arctan2(-n[1], -n[0]), np.arctan2(n[1], n[0])
    segs = [OL.Seg("line", e0 + n, e1 + n), OL.Seg("arc", e1 + n, e1 - n, e1, w / 2, -np.pi),
            OL.Seg("line", e1 - n, e0 - n), OL.Seg("arc", e0 - n, e0 + n, e0, w / 2, -np.pi)]
    from shapely.geometry import LineString
    poly = LineString([e0, e1]).buffer(w / 2, quad_segs=32)
    if OL.polygon(segs).symmetric_difference(poly).area > 0.1 * poly.area:
        segs[1].sweep = segs[3].sweep = np.pi
    rms = float(np.sqrt(np.mean(((P - c0) @ np.array([-d[1], d[0]])) ** 2)))
    return OL.polygon(segs), segs, dict(kind="line", width=float(w), length=float(L),
                                        angle_deg=float(np.degrees(np.arctan2(d[1], d[0]))), skeleton_rms=rms, R=float("inf"))


strokes = {}
# The face is drawn slightly turned, so the face details follow the artwork as drawn: raw masks, no mirror.
for k in ("GLASSES", "NOSE"):
    polys, rings, rep = [], [], []
    g = trace(Mk[k + "_RAW"], small=True)
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        poly, rg, r = OL.build_geom(p, m, None); polys.append(poly); rings += rg; rep += r
    parts[k] = unary_union(polys); SEGS[k] = rings; edges[k] = rep
for k in ("EYEBROWS", "MOUTH"):
    lb, nl = ndi.label(Mk[k + "_RAW"]); polys, rings, info = [], [], []
    comps = sorted([lb == j for j in range(1, nl + 1)], key=lambda c: (X[c].mean() if k == "EYEBROWS" else Y[c].mean()))
    for cm in comps:
        if (cm.sum() / U ** 2) < 1.0: continue
        straight = k == "MOUTH" and not polys          # upper lip (edge 10): straight line, user round 4
        poly, segs, inf = line_stroke(cm) if straight else arc_stroke(cm, False)
        info.append(inf); polys.append(poly); rings.append((segs, []))
    parts[k] = unary_union(polys); SEGS[k] = rings; strokes[k] = info
    edges[k] = [("outer", "stroke", "line-stroke" if inf.get("kind") == "line" else "arc-stroke", 4, 0.0, r[0])
                for r, inf in zip(rings, info)]
    for inf in info: print(f"{k} stroke: {inf.get('kind', 'arc')} R={inf['R']:.1f}px width={inf['width']:.2f}px ({inf['width'] * mm_px:.2f} mm) skeleton rms={inf['skeleton_rms']:.2f}px")

# ---------------- 10. BASE: rectangle ----------------
bm = (lab != names.index("bg")) & (Y >= sole_y)
bx0, bx1 = X[bm].min() - 0.5 / U, X[bm].max() + 0.5 / U
by1 = Y[bm].max() + 0.5 / U
base_hw = (bx1 - bx0) / 2
parts["BASE"] = box(AX - base_hw, sole_y, AX + base_hw, by1)
print(f"base: centre x={(bx0 + bx1) / 2:.2f} (axis {AX:.2f}), width {bx1 - bx0:.2f}px, height {by1 - sole_y:.2f}px")

fits = dict(fist=FIST, boot=BOOT_FIT, arms=dict(outer=o, inner=i, hand_side_x=HX, hand_side_top=H_TOP, hand_side_bottom=H_BOT,
                      ellipse_arcs=ARM_ELLIPSE_ARCS, blend_dev_px=[dev_o, dev_i], blend_from_dx=T_W),
            strokes=strokes, chin=dict(cy=cy, R=R, rms=rms, cx_fit=cx), cravat=dict(w_ref=CQ[0], y_ref=y_ref, w_v=CQ[1], y_v=CQ[2], y_tip=CQ[3],
            top=NOSE_Y, iou=1 - best.fun), face_hidden=dict(half_width=a_, top=top, cy=cy_h, n=n_, tuck_px=tuck),
            flap_tip_y=flap_tip_y, jacket=dict(half_width=T_W, top=NOSE_Y), base=dict(half_width=base_hw, top=sole_y, bottom=by1, artwork_centre_x=(bx0 + bx1) / 2))
F.update(fits); json.dump(F, open(C.WORK / "fits.json", "w"), indent=1, default=float)
pickle.dump(dict(parts=parts, edges=edges, segs=SEGS, metrics=m, AX=AX, sole_y=sole_y), open(C.WORK / "parts_px.pkl", "wb"))
for k, rep in edges.items():
    kinds = {}
    for r in rep: kinds[r[2]] = kinds.get(r[2], 0) + 1
    print(f"{k:9s} runs {kinds}  segments {sum(r[3] for r in rep)}  max dev {max((r[4] for r in rep), default=0):.2f}px")
