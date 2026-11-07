---
name: figure-front-view
description: >
  Reverse-engineer the front view of a flat (2.5D) toy, figurine, desk figure, acrylic/3D-printed
  character standee or similar multi-colour plate into a clean, layered, dimensioned CAD drawing
  (SVG + DXF with one layer per part, true splines) from a straight-on artwork/product image plus
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
> - Reusable code: `pipeline/figfront/` (tested). Worked project scripts: `examples/helmet-guy/`

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

```
understand -> part spec -> sources & authority -> pixel calibration -> colour unmixing
   -> per-part shapes (structure only) -> partition & stacking -> trace visible edges
   -> hidden-part completion -> bounded smoothing + real-corner rebuild
   -> exact geometry where the print is geometric -> verify (numbers + pixel overlays)
   -> export (DXF splines per layer, visible-region DXF, SVG, review page) -> user review loop
```

| Phase | Do | Never |
|---|---|---|
| **1. Understand** | Write `part_spec.md`: object type, how it is made, every part (incl. accessories), its colour, a primitive, its relations, and the **stacking order** read from seams on the print. Show it to the user. | Start tracing pixels before the parts and their order are agreed. |
| **2. Sources & authority** | Decide per question which source wins (artwork = shape; print photo = construction, smoothness, hidden sizes; measurement = scale). Write it down. Ask the user when they conflict. | Average two sources silently. |
| **3. Calibrate the pixel** | Measure nominal mm/px, edge spread vs a simulated ideal edge -> effective pixel; edge noise. Derive tolerance = 1/4 effective px, corner scale = 1 effective px (`figfront.calibrate`). | Hand-pick a smoothing tolerance. |
| **4. Colour unmixing** | Flat-paint palette; each pixel = a paint or a blend of two paints, labelled by majority (`figfront.unmix`). | HSV thresholds (fringes, bumps, false third colour at blends). |
| **5. Shapes for structure** | Fit simple primitives per part only to *partition* same-coloured regions (e.g. arm ring vs jacket, both black) and to *complete hidden edges*. | Let a primitive replace a visible traced edge (kills pose/face detail). |
| **6. Trace + defects** | Sub-pixel contours; round (disk) morphology only; specks never attached at the figure/background edge; bounded spline smoothing; rebuild real corners (straight + non-bending sides). | Diamond morphology; corners on round tips; smoothing beyond the measured tolerance. |
| **7. Exact geometry** | Where the **print** shows clean geometry (oval ring, straight sides, straight V, circular chin, sharp corners from tangents, continuous spline edges), use exact shapes fitted to the pixels. | Apply geometry the print does not show, or to parts the user did not ask about. |
| **8. Verify** | Per-part overlays on the zoomed pixels, overlap scores, max shift in mm, slivers/gaps check, printability (< 0.8 mm features). | Trust a score without looking at the pixel overlay. |
| **9. Review loop** | Publish a review page (overlay slider, dimensioned view, parts sheet). Change **only** what the user points at; show before/after at pixel level. | "Improve" things that were not asked (the user will notice and lose trust). |

## Golden rules (each one cost a review round last time)

1. **Change only what was asked.** When fixing the chin, do not touch the forehead. Diff untouched parts before publishing.
2. **Visible detail is sacred; defects are measured.** Real features down to ~1 effective pixel (a notch between two fists, thumb-base corners) must survive. The defect budget comes from the image, not from taste.
3. **Confirm structural words.** "Arms made from two ovals" meant ONE ring = outer oval minus inner oval, not two mirrored loops. If a description has two plausible geometries, sketch both or ask.
4. **The print decides construction.** One piece or two (jacket + pants = one suit), what is in front (arm ring over jacket, helmet over face), how far hidden parts extend (face continues under the helmet).
5. **Hidden parts follow anatomy/mechanics, visible parts follow the image.** e.g. face: hidden forehead/temples = normal face shape; visible chin = circle measured on the image.
6. **Same-colour neighbours contaminate fits.** The white shirt under a white chin pulled the chin arc down; exclude those columns/regions before fitting.
7. **Ask the user for the one number you cannot see** (a real dimension, plate thickness, slot depth) instead of carrying an assumption forever.

## Using the code

```bash
pip install -e skills/figure-front-view/pipeline        # numpy scipy opencv-python-headless scikit-image shapely ezdxf
python -m pytest skills/figure-front-view/pipeline/tests -q
```

```python
from figfront import unmix, calibrate, contours, geom, layers, export
lab, names = unmix.classify(unmix.upsample(crop, 6), palette, blur_px=0.3, scale=6)
m = calibrate.metrics(mm_per_px, measured_edge_width_px, noise_px)        # -> tolerance, corner scale
poly = contours.mask_to_polygon(part_mask, m, scale=6, origin=(x0, y0))   # bounded smoothing + corners
vis = layers.visible_regions(parts, order_back_to_front)                  # gap-free colour regions
export.write_dxf(parts_mm, "front_view_parts.dxf", aci={...})             # splines split at corners
```

`examples/helmet-guy/` holds the full project scripts (fit -> trace -> smooth -> export -> review
page) as a worked record. They have hard-coded paths/coordinates for that one toy; copy the
structure, not the numbers. See `references/lessons-learned.md` -> "Try next time" for turning
them into a config-driven pipeline.

## Deliverables

- `part_spec.md` (understanding, parts, stacking order, sources/authority, per-part edge rules)
- `*_front_view.svg/.png` dimensioned assembled view; `parts_sheet.svg` each part alone
- `*_parts.dxf` complete parts, one layer each (hidden portions included), true splines
- `*_print_regions_visible.dxf` visible, non-overlapping colour regions for multi-colour printing
- `parts_mm.json`, `pixel_metrics.json`, per-feature fit JSONs; a review page (Artifact) with an
  overlay slider over the artwork pixels

## Scope and ethics

Characters are usually someone else's IP (here: Vinc Studio). Keep outputs for personal use, do not
commit the artwork or derived character outlines to public repos, and say so to the user.
