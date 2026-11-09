---
name: figure-front-view
description: >
  Reverse-engineer the front view of a flat (2.5D) toy, figurine, desk figure, acrylic/3D-printed
  character standee or similar multi-colour plate into a clean, layered, dimensioned CAD drawing
  (SVG + DXF with one layer per part, lines and tangent arcs) from a straight-on artwork/product image plus
  photos of the real object. Use this whenever the user wants to "reverse engineer this toy",
  "make a front view / vector / DXF of this figure", recreate a character standee or keychain
  from a picture, separate a figure into printable colour parts or accessories (helmet, glasses,
  jacket, boots...), or clean up traced outlines of a character for 3D printing / laser cutting -
  even if they just paste a product image and a phone photo. It understands the object first
  (part spec, stacking order from seams on the print), fits shapes per part, traces visible edges
  with colour unmixing, removes ONLY raster defects using a tolerance measured from the image's
  real pixel size, rebuilds sharp corners, and uses exact geometry where the print is geometric.
  Not for full 3D sculpted figures from many photos (use photogrammetry-3d), not for measuring
  mechanical parts with a ruler/ChArUco board (use dim-diagram), and not for recreating someone
  else's character for sale.
---

# Toy / Figure Front View Reverse Engineering (`figure-front-view`)

> **Quick navigation**
> - Full technical method (unmixing, calibration, partition, hidden parts, smoothing, export) -> `references/method.md`
> - **What worked, what went wrong, what to try next time (read before starting)** -> `references/lessons-learned.md`
> - How to run the review loop with the user (screenshots, boxes, numbered marks) -> `references/working-with-the-user.md`
> - Part-spec template + a complete worked example -> `references/part-spec-template.md`
> - Research behind regularization, layering and self-critique -> `references/research.md`
> - Reusable code: `pipeline/figfront/` (tested). Worked project scripts: `examples/fan-figure/` (config-driven, lines + tangent arcs; start here), `examples/helmet-guy/` (first project)

## When this applies

Flat character plates: a figure printed/cut as a constant-thickness plate (often multi-colour FDM,
acrylic, or layered), standing in a base. The deliverable is a **front view** that is faithful to
the character (pose, face, accessories) and clean enough to build from: one closed outline per part,
a stacking order, and gap-free visible colour regions.

Typical inputs:
1. **Straight-on artwork** (product sheet / render). Best for shape intent and colour regions, but it
   is an illustration - low resolution, anti-aliased, compression noise, and it can differ from the
   real product.
2. **Photos of the real print.** Truth for construction: what is one piece, which part is in front,
   which edges are really straight/oval/smooth, how big hidden pieces are. Perspective and blur.
3. **One real measurement** (or a scale object such as a thumb/coin) for millimetres.

## The method in one screen

Goal: **the final quality in <= 3 user rounds.** The first project took ~27 rounds; 23 of them were
things the pipeline could have caught itself (see `references/lessons-learned.md` section 5). The second
took 4 rounds after round 0; each one was a small drawn feature lost in code (section 6). So do all of
the work below BEFORE the user sees anything, then ask once.

```
ROUND 0 (no user)  understand -> part spec + construction rules -> pixel calibration -> colour unmixing
                   -> structure fit -> partition & stacking -> trace -> hidden completion
                   -> regularize (simplest curve per edge) -> exact geometry where the print shows it
                   -> self-review: checks.* + question list on zoomed overlays -> fix -> repeat
ROUND 1 (user)     part spec, stacking order, authority table, open questions        (one message)
ROUND 2 (user)     one review page + numbered edge sheet; user answers by number    (one message)
ROUND 3 (user)     touch-ups only, each passing the scope guard
```

| Phase | Do | Never |
|---|---|---|
| **1. Understand** | Write `part_spec.md`: object, construction, every part (incl. accessories), colour, primitive, relations, **stacking order** from seams on the print. Apply the construction rules (below). | Trace pixels before parts and order are clear. |
| **2. Sources & authority** | Per question: artwork = visible shape; print photo = construction, smoothness, hidden sizes; measurement = scale. Note conflicts for round 1. | Average sources silently. |
| **3. Calibrate the pixel** | `calibrate.metrics`: effective pixel, noise -> fidelity tolerance 1/4 eff px, regularity tolerance 1/2 eff px, corner scale 1 eff px. | Hand-pick tolerances. |
| **4. Colour unmixing** | `unmix.classify` (pixel = paint or two-paint blend). | HSV thresholds. |
| **5. Structure + layers** | Primitives only to partition same-coloured regions and complete hidden edges; back parts continue well under front parts. | Let a primitive replace a visible edge. |
| **6. Trace + outline** | Sub-pixel trace, disk morphology, `contours.drop_specks` (islands/pinholes below max(0.3 mm², (2 eff px)²) unless listed as a small feature), then `outline.build` / `outline.symmetric_part`: corners where the boundary breaks, each run = line, one arc, or tangent arcs (G1). Symmetric parts: fit the right half, mirror it. | Smooth beyond tolerance; corners on round tips; line/arc pieces joined with kinks; leave raster specks in a layer. |
| **6b. Small drawn features** | Steps of 1-4 px (ankles, cuffs): `fine_zone` corners or measured staircase (`pixels`). Touching shapes (two fists): one arc each, sharp notch. Thin lines: `strokes.line_stroke` / `arc_stroke`. Drawn asymmetry (`checks.mirror_offset`): keep details as drawn. | Let the 1-eff-px corner test, smoothing or mirror averaging erase them. |
| **7. Exact geometry** | Where the print shows clean geometry: ring = outer - inner oval, straight panels, straight V, circular chin, shared edges between neighbours. | Geometry the print does not show. |
| **8. Self-review** | `checks.changed_parts`, `changed_outside`, `overlap_report`, `gaps_and_slivers`, `thin_features`, `same_colour_neighbours`, `specks`, `mirror_offset`, `outline.tangent_breaks`; then answer `checks.SELF_REVIEW_QUESTIONS` on zoomed pixel overlays (16-20x for small features). Fix and repeat. | Show the user anything that fails a check. |
| **9. Ask once** | Round 1 questions; round 2 numbered sheet (`review.review_sheet`) on a published review page. | Iterate edge by edge with the user. |

## Construction rules for flat multi-colour figures (defaults; confirm in round 1)
- Every colour region is a body; same-coloured neighbours with no seam on the print are ONE piece (jacket + pants + boots may be one suit).
- A part behind another continues well under it (>= 1.5-2 mm or to a landmark): face = full head under hair/helmet; jacket up behind the face to about the nose; shirt up behind the V-neck.
- A loop/ring is ONE part (outer minus inner) unless the print shows two pieces; parts shown moving stay separate.
- Visible outlines follow the image; hidden outlines follow anatomy/mechanics and must never show.
- Neighbours share one geometry along their boundary (jacket V-neck = cravat V).
- Parts that move together join exactly: arm ends finish on the hand side edges (arms + hands = one closed loop), not a ring running on behind the hands. Where the artwork is black on black (ring over jacket) it holds no edge: keep the visible edge exact and build a tangent join there.
- A moving part uncovers what it hides at rest: those hidden outlines must be finished geometry (the jacket panel behind a turning arm ring).

## Golden rules (each one cost a review round last time)

1. **Change only what was asked.** When fixing the chin, do not touch the forehead. Diff untouched parts before publishing.
2. **Visible detail is sacred; defects are measured.** Real features down to ~1 effective pixel (a notch between two fists, thumb-base corners) must survive. The defect budget comes from the image, not from taste.
3. **Confirm structural words.** "Arms made from two ovals" meant ONE ring = outer oval minus inner oval, not two mirrored loops. If a description has two plausible geometries, sketch both or ask.
4. **The print decides construction.** One piece or two (jacket + pants = one suit), what is in front (arm ring over jacket, helmet over face), how far hidden parts extend (face continues under the helmet).
5. **Hidden parts follow anatomy/mechanics, visible parts follow the image.** e.g. face: hidden forehead/temples = normal face shape; visible chin = circle measured on the image.
6. **Same-colour neighbours contaminate fits.** The white shirt under a white chin pulled the chin arc down; exclude those columns/regions before fitting.
7. **Ask the user for the one number you cannot see** (a real dimension, plate thickness, slot depth) instead of carrying an assumption forever.
8. **Short drawn steps are corners.** A 2-4 px ankle ledge failed the corner test (sides >= 1 effective px) and was smoothed into an S-curve; the user marked it twice. Measure such steps on the pixels and build them square.
9. **Touching shapes keep their notch.** Two fists side by side = two arcs meeting at a sharp notch, even when the notch is only 0.6 px deep. One continuous curve was rejected.
10. **Symmetry is per part, decided by measurement.** Body, helmet and face outline mirrored; face details drawn on a turned face kept as drawn (`mirror_offset` > 0.5 eff px). Mirror averaging of offset thin strokes shrank and moved them.
11. **Scope holds inside the edited part too.** Fixing the boots refit the whole leg run and moved the hip 0.3 mm. Keep the approved segments, splice only the asked zone, prove it with `changed_outside`.

## Using the code

```bash
pip install -e skills/figure-front-view/pipeline        # numpy scipy opencv-python-headless scikit-image shapely ezdxf
python -m pytest skills/figure-front-view/pipeline/tests -q
```

```python
from figfront import unmix, calibrate, contours, geom, layers, export, regularize, outline, strokes, pixels, checks, review
lab, names = unmix.classify(unmix.upsample(crop, 6), palette, blur_px=0.3, scale=6)
m = calibrate.metrics(mm_per_px, measured_edge_width_px, noise_px)        # -> tolerance, corner scale
poly = contours.mask_to_polygon(part_mask, m, scale=6, origin=(x0, y0))   # bounded smoothing + corners
vis = layers.visible_regions(parts, order_back_to_front)                  # gap-free colour regions
poly, rings, rep = outline.symmetric_part(poly, m, axis_x)              # lines + tangent arcs, mirrored half
segs, rep, poly = outline.build(ring, m, fine_zone=(y0, y1))             # + square corners on short steps
p, segs, info = strokes.arc_stroke(mask, X, Y, 6)                         # eyebrow / lip as constant-width stroke
x = pixels.row_crossing(pixels.coverage(art, black, bg), row, x0, x1)     # 50 % edge of a 1-4 px step
dx, dy = checks.mirror_offset(detail_mask, axis_x, 6)                     # drawn asymmetry -> keep as drawn?
flags = checks.overlap_report(parts, order, min_overlap=2.0)              # self-review numbers
rows = review.review_sheet(art_crop, parts, edge_dict, origin=(x0, y0))   # numbered sheet for the user
export.write_dxf(parts_mm, "front_view_parts.dxf", aci={...})             # splines split at corners
```

`examples/helmet-guy/` holds the full project scripts (fit -> trace -> smooth -> export -> review
page) as a worked record. They have hard-coded paths/coordinates for that one toy; copy the
structure, not the numbers. See `references/lessons-learned.md` -> "Try next time" for turning
them into a config-driven pipeline.

## Deliverables

- `part_spec.md` (understanding, parts, stacking order, sources/authority, per-part edge rules)
- `*_front_view.svg/.png` dimensioned assembled view; `parts_sheet.svg` each part alone
- `*_parts.dxf` complete parts, one layer each (hidden portions included), as LINE / ARC / ELLIPSE entities
- `*_print_regions_visible.dxf` visible, non-overlapping colour regions for multi-colour printing
- `parts_mm.json`, `pixel_metrics.json`, per-feature fit JSONs; a review page (Artifact) with an
  overlay slider over the artwork pixels

## Scope and ethics

Characters are usually someone else's IP (here: Vinc Studio). Keep outputs for personal use, do not
commit the artwork or derived character outlines to public repos, and say so to the user.
