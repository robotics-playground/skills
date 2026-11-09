"""mm transform, self-review checks, exports (DXF lines/arcs/ellipses, visible-region DXF, SVG, JSON),
numbered edge sheet. Origin: figure axis at x = 0, sole line (base top) at y = 0, y up, millimetres."""
import json
import pickle
import cv2
import ezdxf
import numpy as np
from shapely import affinity
from shapely.geometry import Polygon, box, mapping
from shapely.ops import unary_union
from figfront import checks, layers, export
import config as C
from figfront import outline as OL

D = pickle.load(open(C.WORK / "parts_px.pkl", "rb"))
F = json.load(open(C.WORK / "fits.json"))
Mk = dict(np.load(C.WORK / "masks.npz"))
parts_px, SEGS, m = D["parts"], D["segs"], D["metrics"]
AX, SOLE = D["AX"], D["sole_y"]
S = m["nominal_mm_per_px"]
U = C.UP
ORDER = C.ORDER
ALL = ["BASE"] + ORDER


def to_mm_g(g):
    g = affinity.translate(g, -AX, -SOLE)
    return affinity.scale(g, S, -S, origin=(0, 0))


def to_mm_p(p):
    return np.array([(p[0] - AX) * S, -(p[1] - SOLE) * S])


parts = {k: to_mm_g(g) for k, g in parts_px.items()}

# ---------------- exact-geometry segment lists (not built by the outline builder) ----------------
o, i = F["arms"]["outer"], F["arms"]["inner"]
# ARMS ellipse arcs over the top: px angle phi (y down) -> DXF parameter t = -phi (y up), counter-clockwise
ELLIPSES = {"ARMS": [((0.0, -(a["e"]["cy"] - SOLE) * S), a["e"]["rx"] * S, a["e"]["ry"] * S, -a["phi_from"], np.pi + a["phi_from"])
                     for a in F["arms"]["ellipse_arcs"]]}
LINES = {}
for k in ("CRAVAT", "BASE"):
    c = np.array(parts[k].exterior.coords)
    LINES[k] = [(c[j], c[j + 1]) for j in range(len(c) - 1)]

# ---------------- self-review checks ----------------
colour = {k: C.COLOUR[k] for k in ALL}
vis = layers.visible_regions(parts, ALL)
silhouette = unary_union(list(parts.values()))
speck_min = max(0.3, (2 * m["effective_source_pixel_mm"]) ** 2)
rep = dict(
    overlap_under_front_parts=checks.overlap_report(parts, ALL, min_overlap=1.5),
    gaps_and_slivers=checks.gaps_and_slivers(vis, silhouette, 0.3),
    thin_features_lt_0_8mm=checks.thin_features(parts, 0.8),
    same_colour_neighbours=checks.same_colour_neighbours(parts, colour, 0.05),
    specks=checks.specks(parts, speck_min),
)

# fidelity: visible colour regions vs the symmetrised artwork classes (raster IoU, working grid)
Hg, Wg = Mk["fg"].shape


def raster_px(g):
    mm_ = np.zeros((Hg, Wg), np.uint8)
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        cv2.fillPoly(mm_, [(np.array(p.exterior.coords) * U - 0.5).round().astype(np.int32)], 1)
        for h in p.interiors: cv2.fillPoly(mm_, [(np.array(h.coords) * U - 0.5).round().astype(np.int32)], 0)
    return mm_.astype(bool)


vis_px = layers.visible_regions(parts_px, ALL)
gy = (np.arange(Hg)[:, None] + 0.5) / U
above = np.broadcast_to(gy < SOLE, (Hg, Wg))
cls_art = {"blue": Mk["HELMET"], "white": Mk["light"] & Mk["fg"],
           "black": Mk["fg"] & ~Mk["HELMET"] & ~(Mk["light"] & Mk["fg"])}
iou = {}
for col in ("blue", "white", "black"):
    a = np.zeros((Hg, Wg), bool)
    for k in ORDER:
        if C.COLOUR[k] == col: a |= raster_px(vis_px[k])
    a &= above; b = cls_art[col] & above
    iou[col] = round(float((a & b).sum() / max((a | b).sum(), 1)), 4)
sil_model = raster_px(unary_union([parts_px[k] for k in ORDER])) & above
iou["silhouette"] = round(float((sil_model & Mk["fg"]).sum() / max((sil_model | Mk["fg"]).sum(), 1)), 4)
rep["fidelity_iou_vs_symmetrised_artwork"] = iou


# tangent continuity at segment joins that are not corners
def kinks(segs):
    out = []
    for a, b in zip(segs, segs[1:] + segs[:1]):
        if np.linalg.norm(a.b - b.a) > 1e-6: continue
        pa = a.points(0.01); pb = b.points(0.01)
        ta = pa[-1] - pa[-2]; tb = pb[1] - pb[0]
        ang = np.degrees(np.arccos(np.clip(ta @ tb / (np.linalg.norm(ta) * np.linalg.norm(tb)), -1, 1)))
        out.append(ang)
    return out


seg_stats = {}
for k, rings in SEGS.items():
    allk = [x for so, hs in rings for ring in [so, *hs] for x in kinks(ring)]
    n_line = sum(s.kind == "line" for so, hs in rings for ring in [so, *hs] for s in ring)
    n_arc = sum(s.kind == "arc" for so, hs in rings for ring in [so, *hs] for s in ring)
    soft = [a for a in allk if a < 20]          # joins below 20 deg are meant to be tangent
    seg_stats[k] = dict(lines=n_line, arcs=n_arc, corners=int(sum(a >= 20 for a in allk)),
                        max_tangent_break_deg=round(max(soft, default=0), 3))
rep["segments"] = seg_stats
json.dump(rep, open(C.OUT / "checks.json", "w"), indent=1, default=str)
print(json.dumps(rep, indent=1, default=str))

# ---------------- part table ----------------
table = {}
for k in ALL:
    g = parts[k]; x0, y0, x1, y1 = g.bounds
    table[k] = dict(colour=C.COLOUR[k], width_mm=round(x1 - x0, 2), height_mm=round(y1 - y0, 2),
                    area_mm2=round(g.area, 1), visible_area_mm2=round(vis[k].area, 1),
                    bounds_mm=[round(v, 2) for v in (x0, y0, x1, y1)])
fig = unary_union([parts[k] for k in ORDER])
dims = dict(figure_height_mm=round(fig.bounds[3], 2), arm_span_mm=round(parts["ARMS"].bounds[2] - parts["ARMS"].bounds[0], 2),
            helmet_width_mm=round(parts["HELMET"].bounds[2] - parts["HELMET"].bounds[0], 2),
            base_mm=[round(parts["BASE"].bounds[2] - parts["BASE"].bounds[0], 2), round(-parts["BASE"].bounds[1], 2)])
fits_mm = dict(
    scale_mm_per_artwork_px=S, scale_source=f"figure height {C.FIGURE_HEIGHT_MM} mm (provisional)",
    arm_ends=dict(meet_hand_side_x_mm=round((F["arms"]["hand_side_x"] - AX) * S, 2),
                  blend_starts_at_x_mm=round(F["arms"]["blend_from_dx"] * S, 2),
                  blend_max_departure_from_oval_mm=[round(v * S, 2) for v in F["arms"]["blend_dev_px"]]),
    arm_ring=dict(outer_half_axes_mm=[round(o["rx"] * S, 2), round(o["ry"] * S, 2)],
                  inner_half_axes_mm=[round(i["rx"] * S, 2), round(i["ry"] * S, 2)],
                  outer_centre_y_mm=round(-(o["cy"] - SOLE) * S, 2), inner_centre_y_mm=round(-(i["cy"] - SOLE) * S, 2)),
    chin_circle_radius_mm=round(F["chin"]["R"] * S, 2),
    fist_radius_mm=round(F["fist"]["radius"] * S, 2),
    boot_mm={k: (np.round(np.array(v) * S, 2).tolist() if isinstance(v, list) else round(v * S, 2)) for k, v in F["boot"].items() if k != "outer_step_line"},
    jacket_width_mm=round(2 * F["T_W"] * S, 2),
    cravat=dict(width_at_chin_mm=round(2 * F["cravat"]["w_ref"] * S, 2), width_at_v_mm=round(2 * F["cravat"]["w_v"] * S, 2),
                v_depth_mm=round((F["cravat"]["y_tip"] - F["cravat"]["y_v"]) * S, 2)),
    strokes={k: [dict(radius_mm=round(s["R"] * S, 2), width_mm=round(s["width"] * S, 2)) for s in v] for k, v in F["strokes"].items()},
    legs_moved_onto_body_axis_mm=round((F["AX"] - F["AX_LEGS"]) * S, 2),
    base_centre_offset_in_artwork_mm=round((F["base"]["artwork_centre_x"] - AX) * S, 2),
)
json.dump(dict(units="mm", origin="figure axis x=0, sole line y=0, y up", order_back_to_front=ALL, dims=dims,
               fits=fits_mm, parts=table,
               geometry={k: mapping(g) for k, g in parts.items()}), open(C.OUT / "parts_mm.json", "w"), indent=1, default=float)

# ---------------- DXF: complete parts, LINE / ARC / ELLIPSE ----------------
ACI = {"blue": 5, "white": 8, "black": 7}


def add_seg(msp, s, layer):
    if s.kind == "line":
        a, b = to_mm_p(s.a), to_mm_p(s.b)
        if np.linalg.norm(a - b) > 1e-6: msp.add_line(tuple(a), tuple(b), dxfattribs={"layer": layer})
        return
    c = to_mm_p(s.c)
    ta = -np.degrees(np.arctan2(s.a[1] - s.c[1], s.a[0] - s.c[0]))
    sw = -np.degrees(s.sweep)
    if sw > 0: st, en = ta, ta + sw
    else: st, en = ta + sw, ta
    msp.add_arc(tuple(c), s.r * S, st % 360, en % 360, dxfattribs={"layer": layer})


def write_parts_dxf(path, keys):
    doc = ezdxf.new(units=4); msp = doc.modelspace()
    for k in keys:
        doc.layers.add(k, color=ACI[C.COLOUR[k]])
        for so, hs in SEGS.get(k, []):
            for ring in [so, *hs]:
                for s in ring: add_seg(msp, s, k)
        for a, b in LINES.get(k, []):
            msp.add_line(tuple(a), tuple(b), dxfattribs={"layer": k})
        for (cx, cy), rx, ry, t0, t1 in ELLIPSES.get(k, []):
            assert rx >= ry
            msp.add_ellipse((cx, cy), major_axis=(rx, 0), ratio=ry / rx, start_param=t0, end_param=t1, dxfattribs={"layer": k})
    doc.saveas(path)


write_parts_dxf(C.OUT / "fan_figure_parts.dxf", ALL)
export.write_dxf({k: vis[k] for k in ALL if not vis[k].is_empty}, str(C.OUT / "fan_figure_print_regions_visible.dxf"),
                 aci={k: ACI[C.COLOUR[k]] for k in ALL})

# ---------------- SVG: assembled view with dimensions + parts sheet ----------------
FILL = {"blue": "#2b8fd0", "white": "#f6f6f4", "black": "#262626"}


def path_d(g):
    return export.svg_path(g, flip_y=True)


xmin, ymin, xmax, ymax = unary_union(list(parts.values())).bounds
pad = 14
W_, H_ = xmax - xmin + 2 * pad + 20, ymax - ymin + 2 * pad
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{xmin - pad} {-ymax - pad} {W_} {H_}" width="{W_ * 4:.0f}" height="{H_ * 4:.0f}" font-family="sans-serif">']
for k in ALL:
    if vis[k].is_empty: continue
    svg.append(f'<path id="{k}" d="{path_d(vis[k])}" fill="{FILL[C.COLOUR[k]]}" stroke="#888" stroke-width="0.08" fill-rule="evenodd"/>')


def dim_h(x0, x1, y, label):
    return (f'<g stroke="#c0392b" stroke-width="0.15" fill="#c0392b" font-size="2.6" text-anchor="middle">'
            f'<line x1="{x0}" y1="{-y}" x2="{x1}" y2="{-y}"/><line x1="{x0}" y1="{-y - 1.2}" x2="{x0}" y2="{-y + 1.2}"/>'
            f'<line x1="{x1}" y1="{-y - 1.2}" x2="{x1}" y2="{-y + 1.2}"/><text x="{(x0 + x1) / 2}" y="{-y - 0.8}" stroke="none">{label}</text></g>')


def dim_v(x, y0, y1, label):
    return (f'<g stroke="#c0392b" stroke-width="0.15" fill="#c0392b" font-size="2.6">'
            f'<line x1="{x}" y1="{-y0}" x2="{x}" y2="{-y1}"/><line x1="{x - 1.2}" y1="{-y0}" x2="{x + 1.2}" y2="{-y0}"/>'
            f'<line x1="{x - 1.2}" y1="{-y1}" x2="{x + 1.2}" y2="{-y1}"/>'
            f'<text x="{x + 1}" y="{-(y0 + y1) / 2}" stroke="none">{label}</text></g>')


hb = parts["HELMET"].bounds; ab = parts["ARMS"].bounds; bb = parts["BASE"].bounds
svg.append(dim_v(xmax + 6, 0, fig.bounds[3], f"{fig.bounds[3]:.1f}"))
svg.append(dim_v(xmax + 6, bb[1], 0, f"{-bb[1]:.1f}"))
svg.append(dim_h(ab[0], ab[2], ab[3] + 3, f"arm ring {ab[2] - ab[0]:.1f}"))
svg.append(dim_h(hb[0], hb[2], hb[3] + 3, f"helmet {hb[2] - hb[0]:.1f}"))
svg.append(dim_h(bb[0], bb[2], bb[1] - 4, f"base {bb[2] - bb[0]:.1f}"))
svg.append(f'<line x1="0" y1="{-ymax - pad}" x2="0" y2="{-ymin + pad}" stroke="#7f8c8d" stroke-width="0.12" stroke-dasharray="1.5 1"/>')
svg.append(f'<text x="{xmin}" y="{-ymin + pad - 2}" font-size="2.4" fill="#555">mm, provisional scale (figure height {C.FIGURE_HEIGHT_MM} mm)</text>')
svg.append("</svg>")
open(C.OUT / "fan_figure_front_view.svg", "w").write("\n".join(svg))

# parts sheet: each complete part alone (hidden portions included), with size
cells = []; x = 0
for k in ALL:
    g = parts[k]; b = g.bounds; w = b[2] - b[0]
    gg = affinity.translate(g, x - b[0], 0)
    cells.append(f'<path d="{path_d(gg)}" fill="{FILL[C.COLOUR[k]]}" stroke="#555" stroke-width="0.15" fill-rule="evenodd"/>'
                 f'<text x="{x}" y="{-b[1] + 5}" font-size="3">{k} {w:.1f} x {b[3] - b[1]:.1f}</text>')
    x += max(w, 32) + 8
svg2 = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-5 -105 {x + 5} 140" width="{(x + 5) * 3:.0f}" height="420" font-family="sans-serif">',
        '<rect x="-5" y="-105" width="100%" height="140" fill="#e9ecef"/>', *cells, "</svg>"]
open(C.OUT / "fan_figure_parts_sheet.svg", "w").write("\n".join(svg2))
print("dims", dims)

# ---------------- numbered edge sheet (right half numbered; the left half is its mirror) ----------------
from render import art_corrected
SC = 5
img = art_corrected(SC)
img = (img.astype(np.float32) * 0.45 + 140).astype(np.uint8)
KCOL = {"line": (0, 150, 0), "arc": (200, 0, 200), "curve": (220, 90, 0), "arc-stroke": (0, 140, 220), "line-stroke": (0, 140, 220),
        "ellipse": (0, 0, 220), "exact-line": (0, 110, 110)}
rows = []


def put(pts_px, kind, part, note, extra=None):
    n = len(rows) + 1
    P = (np.asarray(pts_px) * SC).round().astype(np.int32)
    cv2.polylines(img, [P], False, KCOL[kind], 2, cv2.LINE_AA)
    cv2.circle(img, tuple(int(v) for v in P[0]), 3, (0, 0, 0), -1); cv2.circle(img, tuple(int(v) for v in P[-1]), 3, (0, 0, 0), -1)
    mid = P[len(P) // 2]
    L = float(np.sum(np.linalg.norm(np.diff(np.asarray(pts_px), axis=0), axis=1))) * S
    rows.append(dict(id=n, part=part, kind=kind, length_mm=round(L, 1), note=note, **(extra or {})))
    return n, mid


labels = []
for part in ["HELMET", "FACE", "GLASSES", "NOSE", "EYEBROWS", "MOUTH", "HANDS", "BODY"]:
    for r in D["edges"][part]:
        segs = r[5]
        if not segs: continue
        pts = np.vstack([s.points(0.2) for s in segs])
        if np.median(pts[:, 0]) < AX - 0.5 and part not in ("GLASSES", "NOSE", "EYEBROWS", "MOUTH"): continue  # mirrored half
        kind = r[2]
        arcs = [s for s in segs if s.kind == "arc"]
        extra = dict(segments=len(segs))
        if kind == "arc" and arcs: extra["radius_mm"] = round(float(np.mean([s.r for s in arcs])) * S, 2)
        note = {"line": "straight", "arc": "one circular arc", "curve": f"smooth curve, {len(segs)} tangent arcs/lines",
                "arc-stroke": "constant-width arc stroke, round ends", "line-stroke": "straight constant-width stroke, round ends"}.get(kind, kind)
        n, mid = put(pts, kind, part, note, extra); labels.append((n, mid))
# exact geometry
for nm, a in zip(("outer oval", "inner oval"), F["arms"]["ellipse_arcs"]):
    e = a["e"]; t = np.linspace(-np.pi / 2, a["phi_from"], 200)
    pts = np.c_[AX + e["rx"] * np.cos(t), e["cy"] + e["ry"] * np.sin(t)]
    n, mid = put(pts, "ellipse", "ARMS", f"{nm}, exact ellipse (visible part)", dict(half_axes_mm=[round(e['rx'] * S, 2), round(e['ry'] * S, 2)]))
    labels.append((n, mid))
cv_ = np.array(parts_px["CRAVAT"].exterior.coords)
right = cv_[cv_[:, 0] >= AX - 1e-6]
right = right[np.argsort(right[:, 1])]
n, mid = put(right[:2], "exact-line", "CRAVAT", "shirt side, straight (top hidden under face)"); labels.append((n, mid))
n, mid = put(right[1:3], "exact-line", "CRAVAT", "V edge, straight, sharp tip"); labels.append((n, mid))
bp = np.array(parts_px["BASE"].exterior.coords)
n, mid = put(np.array([[AX, SOLE], [bp[:, 0].max(), SOLE], [bp[:, 0].max(), bp[:, 1].max()], [AX, bp[:, 1].max()]]),
             "exact-line", "BASE", "rectangle, centred on figure axis"); labels.append((n, mid))
# new in round 2 (numbers after 31 so round-1 numbers stay valid)
arm_segs = SEGS["ARMS"][0][0]
blends = [sg for sg in arm_segs if sg.kind == "arc" and sg.a[0] > AX and sg.b[0] > AX]
for nm, pair in (("outer arm end: tangent arcs from the oval into the hand corner (bottom of 12)", blends[0:2]),
                 ("inner arm end: tangent arcs from the oval into the hand corner (top of 12)", blends[2:4])):
    n, mid = put(np.vstack([sg.points(0.2) for sg in pair]), "curve", "ARMS", nm, dict(segments=2)); labels.append((n, mid))
for n, mid in labels:
    x, y = int(mid[0]) + 6, int(mid[1]) - 6
    cv2.circle(img, (x, y), 10, (255, 255, 255), -1); cv2.circle(img, (x, y), 10, (0, 0, 0), 1)
    cv2.putText(img, str(n), (x - (7 if n > 9 else 4), y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 1, cv2.LINE_AA)
cv2.line(img, (int(AX * SC), 0), (int(AX * SC), img.shape[0]), (120, 120, 120), 1)
cv2.imwrite(str(C.WORK / "edge_sheet.png"), img)
json.dump(rows, open(C.OUT / "edge_sheet.json", "w"), indent=1)
print(len(rows), "numbered edges")
