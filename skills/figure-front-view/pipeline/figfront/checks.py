"""Automatic self-review: the checks a human reviewer had to do by hand in the first project.

Run these BEFORE showing anything to the user; fix what they flag; then present one consolidated
review sheet (review.py) instead of iterating edge by edge with the user.
"""
from __future__ import annotations
import numpy as np
from shapely.geometry import Point
from shapely.ops import unary_union


def changed_parts(prev: dict, new: dict, allowed: set, tol: float = 1e-3) -> dict:
    """Scope guard: parts not in `allowed` must not move. Returns {part: symmetric-difference area}."""
    out = {}
    for k in set(prev) | set(new):
        if k in allowed: continue
        a, b = prev.get(k), new.get(k)
        if a is None or b is None: out[k] = float('inf'); continue
        d = a.symmetric_difference(b).area
        if d > tol: out[k] = d
    return out


def overlap_report(parts: dict, order: list, min_overlap: float) -> list:
    """For every part covered by a part in front of it: how far does the back part reach under the front
    part along their shared visible boundary? Flags back parts that stop short (e.g. a face that does not
    continue under the helmet, a jacket that stops at the chin). `min_overlap` in the parts' units."""
    flags = []
    for i, back in enumerate(order):
        for front in order[i + 1:]:
            if back not in parts or front not in parts: continue
            B, F = parts[back], parts[front]
            inter = B.intersection(F)
            if B.distance(F) > 1e-6: continue
            touch = B.boundary.intersection(F.buffer(1e-6))
            # overlap depth: largest inscribed reach of B inside F (approx. by negative buffer of the intersection)
            depth = 0.0
            if not inter.is_empty:
                lo, hi = 0.0, max(inter.bounds[2] - inter.bounds[0], inter.bounds[3] - inter.bounds[1])
                for _ in range(20):
                    mid = (lo + hi) / 2
                    if inter.buffer(-mid / 2).is_empty: hi = mid
                    else: lo = mid
                depth = lo
            if depth < min_overlap and touch.length > 0:
                flags.append(dict(back=back, front=front, overlap=round(depth, 3)))
    return flags


def gaps_and_slivers(visible: dict, silhouette, min_area: float) -> dict:
    """Visible colour regions must tile the silhouette: report uncovered area and tiny pieces."""
    u = unary_union(list(visible.values()))
    gap = silhouette.difference(u)
    slivers = {k: [round(p.area, 4) for p in getattr(g, 'geoms', [g]) if 0 < p.area < min_area] for k, g in visible.items()}
    return dict(uncovered_area=round(gap.area, 4), slivers={k: v for k, v in slivers.items() if v})


def thin_features(parts: dict, min_width: float) -> dict:
    """Share of each part narrower than min_width (e.g. 0.8 mm for a 0.4 mm nozzle)."""
    out = {}
    for k, g in parts.items():
        if g.is_empty or g.area == 0: continue
        r = g.buffer(-min_width / 2).buffer(min_width / 2)
        lost = 1 - r.area / g.area
        if lost > 0.25: out[k] = round(lost, 2)
    return out


def same_colour_neighbours(parts: dict, colour: dict, touch_tol: float) -> list:
    """Adjacent parts with the same paint: fits on one can be contaminated by the other (shirt under a
    white chin), and they may really be one piece (jacket + pants)."""
    ks = list(parts); out = []
    for i in range(len(ks)):
        for j in range(i + 1, len(ks)):
            a, b = ks[i], ks[j]
            if colour.get(a) == colour.get(b) and parts[a].distance(parts[b]) <= touch_tol:
                out.append((a, b, colour.get(a)))
    return out


def specks(parts: dict, min_area: float) -> dict:
    """Islands and holes below `min_area` per part (should be empty after contours.drop_specks).
    A part made only of pieces below the threshold is a deliberately small part, not a speck."""
    from shapely.geometry import Polygon as _P
    out = {}
    for k, g in parts.items():
        polys = [p for p in getattr(g, 'geoms', [g]) if not p.is_empty]
        if not polys or all(p.area < min_area for p in polys) and len(polys) == 1: continue
        isl = sum(1 for p in polys if p.area < min_area)
        holes = sum(1 for p in polys for h in p.interiors if _P(h).area < min_area)
        if isl or holes: out[k] = dict(islands=isl, holes=holes, pieces=len(polys))
    return out


SELF_REVIEW_QUESTIONS = [
    "Corners: is every vertex in the source drawn sharp, and are round tips (thumbs, toes) still round?",
    "Continuity: does every long edge read as ONE curve (no wobble) between real corners?",
    "Regularity: are straight runs straight, circular runs circular (chin, ring), V shapes made of lines?",
    "Small drawn features: notches/bulges ~1 effective pixel still present (fists, crotch bulge)?",
    "Construction: does each back part continue well under the part in front (face under helmet, suit up "
    "behind the face, shirt behind the V-neck)? Any visible gap or background sliver at a junction?",
    "Pieces: are same-colour regions without a visible seam on the print merged (jacket + pants = suit)?",
    "Rings/loops: is a loop modelled as ONE ring (outer minus inner) unless the print shows two parts?",
    "Contamination: was any fit pulled by a same-colour neighbour (white shirt under a white chin)?",
    "Scope: did anything change that the user did not ask about (changed_parts), and inside the edited part, "
    "outside the asked zone (changed_outside)?",
    "Printability: any feature thinner than the nozzle allows (thin_features)?",
    "Specks: any part with islands or pinholes below max(0.3 mm^2, (2 effective px)^2) (specks)? Remove them "
    "with contours.drop_specks unless the part spec lists them as a real small feature.",
    "Short steps: do 1-4 px drawn steps (ankle ledges, cuffs, hems) keep square corners on the zoomed overlay, "
    "or did the corner test (sides >= 1 effective px) and smoothing round them into S-curves?",
    "Touching shapes: where two shapes meet (two fists, two feet), is there a notch with a sharp corner, not "
    "one continuous curve?",
    "Drawn asymmetry: does mirror_offset of each face detail stay under ~0.5 effective px? If not, the face is "
    "drawn turned: keep the details as drawn (mirror averaging shrinks thin strokes).",
    "Joins of moving parts: does each attached part end exactly on its partner's edge (arm ends on the hand "
    "sides), with no step and no band running on behind it?",
    "Tangency: is every join below 20 deg truly tangent (tangent_breaks), with real corners well above it?",
]


def changed_outside(prev, new, band_y, tol: float = 1e-3) -> float:
    """Scope guard INSIDE one part: area of change outside the y band (y0, y1) the user asked about.
    A refit of one edge run can move the whole run (fixing a boot moved the leg curve 0.3 mm)."""
    from shapely.geometry import box
    big = 1e6
    out = prev.symmetric_difference(new).difference(box(-big, band_y[0], big, band_y[1]))
    return 0.0 if out.area <= tol else float(out.area)


def mirror_offset(mask, axis, scale):
    """Largest drawn asymmetry of a feature mask (working grid, `scale`x), as (dx, dy) in image px.
    Components crossing the axis: centroid offset from it. Side components: centroid difference to the
    mirrored partner. Above ~0.5 effective px the artist drew it asymmetric (a turned face)."""
    from scipy import ndimage as ndi
    lab, n = ndi.label(mask)
    if n == 0: return (0.0, 0.0)
    idx = range(1, n + 1)
    cents = [np.array(ndi.center_of_mass(lab == i))[::-1] / scale + 0.5 / scale for i in idx]
    sizes = np.asarray(ndi.sum(mask, lab, idx))
    best = (0.0, 0.0)
    for i, c in enumerate(cents):
        xs = np.nonzero(lab == i + 1)[1]; x0, x1 = xs.min() / scale, (xs.max() + 1) / scale
        if x0 < axis < x1:
            off = (c[0] - axis, 0.0)
        elif c[0] > axis:
            mir = np.array([2 * axis - c[0], c[1]])
            others = [cc for j, cc in enumerate(cents) if cc[0] < axis and abs(sizes[j] - sizes[i]) < 0.5 * sizes[i]]
            if not others: continue
            cc = min(others, key=lambda q: np.linalg.norm(q - mir)); off = tuple(cc - mir)
        else:
            continue
        if np.hypot(*off) > np.hypot(*best): best = (float(off[0]), float(off[1]))
    return best
