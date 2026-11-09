"""Symmetric part masks and exact-geometry fits.

Coordinates: artwork px, pixel-edge convention (pixel i covers [i, i+1]); grid pixel j -> (j + 0.5) / UP.
"""
import json
import pickle
import cv2
import numpy as np
from scipy import ndimage as ndi
from scipy.optimize import least_squares
from figfront.contours import disk_op
import config as C

U = C.UP
d = np.load(C.WORK / "labels.npz"); lab = d["lab"]; names = list(d["names"]); sole_y = float(d["sole_y"])
s1 = json.load(open(C.WORK / "stage1.json"))
AX = s1["axes"]["torso+arms"][0] + 0.5 / U          # figure axis (head and torso agree within 0.01 px)
AX_LEGS = s1["axes"]["legs"][0] + 0.5 / U           # legs are drawn 3 px left of the body axis
Hg, Wg = lab.shape
gy, gx = np.mgrid[0:Hg, 0:Wg]
X = (gx + 0.5) / U; Y = (gy + 0.5) / U
cls = lambda n: lab == names.index(n)


def mirror(m, axis):
    a = axis * U - 0.5
    M = np.float32([[-1, 0, 2 * a], [0, 1, 0]])
    return cv2.warpAffine(m.astype(np.float32), M, (Wg, Hg), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)


def shift_x(m, dx):
    M = np.float32([[1, 0, dx * U], [0, 1, 0]])
    return cv2.warpAffine(m.astype(np.float32), M, (Wg, Hg), flags=cv2.INTER_LINEAR)


def symmetric(m, y_split):
    """Average a mask with its mirror image; legs (below y_split) are first moved onto the body axis."""
    m = m.astype(np.float32)
    legs = shift_x(m, AX - AX_LEGS)
    m = np.where(Y >= y_split, legs, m)
    s = 0.5 * (m + mirror(m, AX))
    return cv2.GaussianBlur(s, (0, 0), 0.25 * U) >= 0.5


def largest(m):
    l, n = ndi.label(m)
    if n == 0: return m
    sz = np.bincount(l.ravel()); sz[0] = 0
    return l == sz.argmax()


def comp_at(m, x, y):
    l, _ = ndi.label(m)
    v = l[int(y * U), int(x * U)]
    assert v, f"no component at {(x, y)}"
    return l == v


above_sole = Y < sole_y
# ---- raw classes; grey (hands, shirt shading) and white are one paint on the print ----
light_raw = (cls("white") | cls("grey")) & above_sole
light_raw = disk_op(light_raw, "open", 0.6 * U)        # drop the 0.5 px light halo along black|bg edges
blue_raw = largest(cls("blue") & above_sole)
fg_raw = ~cls("bg") & above_sole
fg_raw = largest(disk_op(fg_raw, "open", 0.5 * U))

# waist split row: below it the legs are symmetrised about their own axis, then moved to the body axis
Y_SPLIT = 176.0
fg = symmetric(fg_raw, Y_SPLIT)
blue = largest(symmetric(blue_raw, Y_SPLIT))
light = symmetric(light_raw, Y_SPLIT) & fg

# ---- head ----
hh, _ = ndi.label(ndi.binary_fill_holes(blue) & ~blue)
sz = np.bincount(hh.ravel()); sz[0] = 0
HELMET = blue | np.isin(hh, np.nonzero((sz > 0) & (sz < (3 * U) ** 2))[0])
face_c = (AX + 15, 100.0)
HEADC = ndi.binary_fill_holes(comp_at(light & ~HELMET, *face_c)) & ~HELMET
# face and shirt are both white and touch at the chin: split where the white region narrows to the shirt
wrow = HEADC.sum(1) / U
rows_below = np.nonzero((np.arange(Hg) / U > face_c[1]) & (wrow < 25) & (wrow > 0))[0]
Y_NECK = (rows_below[0] + 0.5) / U
FACE_VIS = HEADC & (Y < Y_NECK)
SHIRT_TOP = HEADC & (Y >= Y_NECK)
print(f"face/shirt split row y={Y_NECK:.2f}")
# face inlays from the RAW labels: averaging thin strokes with their mirror image shrinks them when the
# artwork draws left and right a little offset, so each class is aligned to its mirror first
black_raw = cls("black") & above_sole
dark_in_face = black_raw & disk_op(FACE_VIS, "dil", 1 * U) & ~disk_op(HELMET, "dil", 1 * U)
dl, dn = ndi.label(dark_in_face)
details = {"GLASSES": np.zeros_like(fg), "EYEBROWS": np.zeros_like(fg), "NOSE": np.zeros_like(fg), "MOUTH": np.zeros_like(fg)}
info = []
for i in range(1, dn + 1):
    m = dl == i; a = m.sum() / U ** 2
    if a < 0.5 or (m & FACE_VIS).sum() < 0.5 * m.sum(): continue      # jacket at the chin edge is not a face line
    cy = Y[m].mean(); cx = X[m].mean(); w = (X[m].max() - X[m].min())
    info.append((round(cx, 1), round(cy, 1), round(a, 1), round(w, 1)))
    if a > 60: k = "GLASSES"
    elif cy < 70: k = "EYEBROWS"
    elif a < 20 and abs(cx - AX) < 3 and cy < 90: k = "NOSE"
    else: k = "MOUTH"
    details[k] |= m
print("face dark components (cx, cy, area px2, width):", info)


def symmetric_aligned(m, search_px=3.0):
    ys, xs = np.nonzero(m); pad = int((search_px + 2) * U)
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad, Hg); 
    full = m.astype(np.float32); mm = mirror(full, AX)
    a = full[y0:y1]; b = mm[y0:y1]; best = (-1, 0, 0); r = int(search_px * U)
    for dy in range(-r, r + 1):
        bb = np.roll(b, dy, 0)
        for dx in range(-r, r + 1):
            sc = (np.roll(bb, dx, 1) * a).sum()
            if sc > best[0]: best = (sc, dx, dy)
    _, dx, dy = best
    sh = lambda img, sx, sy: cv2.warpAffine(img, np.float32([[1, 0, sx], [0, 1, sy]]), (Wg, Hg), flags=cv2.INTER_LINEAR)
    avg = 0.5 * (sh(full, -dx / 2, -dy / 2) + sh(mm, dx / 2, dy / 2))
    return cv2.GaussianBlur(avg, (0, 0), 0.25 * U) >= 0.5, (dx / U, dy / U)


RAW = {k + "_RAW": v.copy() for k, v in details.items()}   # face is drawn slightly turned: details kept as drawn
for k in details:
    details[k], off = symmetric_aligned(details[k])
    print(f"{k}: left/right drawing offset dx={off[0]:.2f}px dy={off[1]:.2f}px")

# ---- body light parts ----
CRAVAT_VIS = SHIRT_TOP
HANDS_VIS = comp_at(light & ~FACE_VIS & ~CRAVAT_VIS, AX, 160.0)
BLACK = fg & ~HELMET & ~FACE_VIS & ~CRAVAT_VIS & ~HANDS_VIS

# ---- lunes = background holes inside the arm ring (left and right of the jacket) ----
holes = ndi.binary_fill_holes(fg) & ~fg
hl, hn = ndi.label(holes)
hs = np.bincount(hl.ravel()); hs[0] = 0
lunes = [hl == i for i in np.argsort(-hs)[:2] if hs[i] > 50 * U * U]
assert len(lunes) == 2, "expected two ring holes"


def boundary_pts(m):
    e = m & ~ndi.binary_erosion(m)
    return np.c_[X[e], Y[e]]


lp = np.vstack([boundary_pts(m) for m in lunes])
dx = np.abs(lp[:, 0] - AX)
# jacket side = the straight inner edge of each lune (rows where the lune is clearly open)
ly = lp[:, 1]; yl0, yl1 = ly.min(), ly.max()
mid_rows = (ly > yl0 + 0.25 * (yl1 - yl0)) & (ly < yl0 + 0.75 * (yl1 - yl0))
inner_side = []
for m in lunes:
    p = boundary_pts(m)
    for yy in np.unique(np.round(p[:, 1] * U)):
        r = p[np.round(p[:, 1] * U) == yy]
        if yl0 + 0.25 * (yl1 - yl0) < r[0, 1] < yl0 + 0.75 * (yl1 - yl0):
            inner_side.append(np.abs(r[:, 0] - AX).min())
T_W = float(np.median(inner_side)) - 0.5 / U          # boundary pixels sit half a grid px inside the hole
print(f"jacket half-width {T_W:.2f} px  (lune rows {yl0:.1f}..{yl1:.1f})")


def fit_ellipse_axis(pts, p0):
    """Ellipse centred on the figure axis: params cy, rx, ry; geometric residual approximation."""
    def res(q):
        cy, rx, ry = q
        u = (pts[:, 0] - AX) / rx; v = (pts[:, 1] - cy) / ry
        r = np.hypot(u, v)
        return (r - 1) * np.hypot(rx * u, ry * v) / np.maximum(r, 1e-9)
    q = least_squares(res, p0, loss="soft_l1", f_scale=0.3).x
    return q, float(np.sqrt(np.mean(res(q) ** 2)))


# inner oval: outer side of each lune
inner_pts = lp[dx > T_W + 1.0]
qi, rms_i = fit_ellipse_axis(inner_pts, [128, 58, 38])
# outer oval: silhouette edge of the ring, away from the helmet and the hips
sil = boundary_pts(ndi.binary_fill_holes(fg))
near_helmet = disk_op(HELMET, "dil", 2 * U)
nh = near_helmet[(sil[:, 1] * U).astype(int), (sil[:, 0] * U).astype(int)]
outer_pts = sil[(np.abs(sil[:, 0] - AX) > T_W + 3) & (sil[:, 1] < Y_SPLIT - 12) & ~nh]
qo, rms_o = fit_ellipse_axis(outer_pts, [128, 95, 58])
print(f"outer oval cy={qo[0]:.2f} rx={qo[1]:.2f} ry={qo[2]:.2f} rms={rms_o:.3f}px (n={len(outer_pts)})")
print(f"inner oval cy={qi[0]:.2f} rx={qi[1]:.2f} ry={qi[2]:.2f} rms={rms_i:.3f}px (n={len(inner_pts)})")

# ---- cravat: straight-line V fitted to the visible shirt (below the chin, above the hands) ----
cv_y = Y[CRAVAT_VIS]
print(f"cravat visible rows {cv_y.min():.2f}..{cv_y.max():.2f}")

out = dict(AX=AX, AX_LEGS=AX_LEGS, Y_SPLIT=Y_SPLIT, sole_y=sole_y, T_W=T_W,
           outer=dict(cy=qo[0], rx=qo[1], ry=qo[2], rms=rms_o), inner=dict(cy=qi[0], rx=qi[1], ry=qi[2], rms=rms_i))
json.dump(out, open(C.WORK / "fits.json", "w"), indent=1, default=float)
masks = dict(fg=fg, HELMET=HELMET, FACE_VIS=FACE_VIS, CRAVAT_VIS=CRAVAT_VIS, HANDS_VIS=HANDS_VIS, BLACK=BLACK,
             light=light, **RAW, **details)
np.savez_compressed(C.WORK / "masks.npz", **masks)
np.savez_compressed(C.WORK / "ring_fit_pts.npz", outer=outer_pts, inner=inner_pts)

# overlay: symmetric class map + fitted ovals + jacket lines
S = 4
art = cv2.imread(str(C.ARTWORK))
im = (cv2.resize(art, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST) * 0.6 + 100).astype(np.uint8)
def ell(q, col):
    t = np.linspace(0, 2 * np.pi, 400)
    P = np.c_[AX + q[1] * np.cos(t), q[0] + q[2] * np.sin(t)] * S
    cv2.polylines(im, [P.round().astype(np.int32)], True, col, 1, cv2.LINE_AA)
ell(qo, (0, 0, 255)); ell(qi, (0, 160, 0))
for sx in (-1, 1):
    x = int(round((AX + sx * T_W) * S)); cv2.line(im, (x, 0), (x, im.shape[0]), (255, 0, 0), 1)
cv2.line(im, (int(AX * S), 0), (int(AX * S), im.shape[0]), (255, 0, 255), 1)
for k, m in masks.items():
    if k in ("fg", "light", "BLACK"): continue
    cs, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    for c in cs: cv2.polylines(im, [(c.reshape(-1, 2) * S / U).round().astype(np.int32)], True, (0, 200, 255), 1)
cv2.imwrite(str(C.WORK / "stage2_overlay.png"), im)
