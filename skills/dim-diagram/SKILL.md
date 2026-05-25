---
name: dim-diagram
description: >
  Turn phone photos of an object into a CAD-style 3-view dimensioned drawing. Use when the user
  wants to measure an object's real dimensions from a photo, generate a 2D dimensional drawing or
  datasheet, extract length/width/height or hole sizes from an image with a ruler or ChArUco/ArUco
  reference, rectify a photo to metric scale, or produce an SVG/PDF/DXF engineering sheet. Covers
  ChArUco board printing, phone capture for accuracy, OpenCV homography rectification, sub-pixel
  measurement, and ezdxf drawing output.
---

# Photo → CAD-style Dimensioned Drawing (`dim-diagram`)

> **Quick navigation**
> - How to shoot the photos for accuracy (phone-first) -> `references/capture-guide.md`
> - ChArUco detection, homography rectification, camera squareness -> `references/rectification.md`
> - Segmentation, sub-pixel edges, model fitting, features -> `references/measurement.md`
> - ezdxf dimensioned sheet, layout, title block, formats -> `references/drawing-output.md`

---

## Who This Skill Is For

This skill is for developers who need **real-world dimensions of an object from photographs** —
a 2D datasheet-style drawing like a manufacturer's mechanical drawing — without a CAD model or
calipers. You shoot **three photos** (front, top, side), each with a printed **ChArUco board**
in frame for scale, and the pipeline produces a dimensioned drawing plus a metric `bundle/`.

The hard truth up front: **accuracy is won at capture time, not in software.** A fiducial board
placed correctly, shot square-on from far away, gets you to ±1–2 %. A ruler tossed on the table
in front of the object can be off by 10 %. Read `references/capture-guide.md` before shooting.

It is **not** a 3D reconstruction tool. For a 3D mesh, the sibling skill `photogrammetry-3d`
consumes this skill's `bundle/` (scale + silhouettes) plus an orbit photo set.

---

## Pipeline Architecture

Each photo is rectified and measured independently, then the three views are reconciled and
drawn. No 3D reconstruction.

```
                 per-view pipeline (run x3: front, top, side)
  photo ─► reference ─► rectify ─► segment ─► subpixel ─► measure ─► ViewMeasurement(JSON)
            (detect     (homography (isolate   (refine    (fit + dimension,
             ChArUco)    -> mm/px)   object)    edges)     with +/- sigma)

                 assembly
  3x ViewMeasurement ─► reconcile ─► draw (ezdxf) ─► drawing.{svg,png,pdf,dxf} + bundle/
                        (shared-dim                                  (for photogrammetry-3d)
                         agreement)
```

| Stage | What happens | Module |
|-------|--------------|--------|
| **reference** | Detect the ChArUco board, sub-pixel corners | `dimgram/reference.py` |
| **rectify** | Homography to a fronto-parallel metric image; tilt warning | `dimgram/rectify.py` |
| **segment** | Isolate the object silhouette (classical; SAM optional) | `dimgram/segment.py` |
| **subpixel** | Refine the contour to sub-pixel — *never measure off a raw mask* | `dimgram/subpixel.py` |
| **measure** | Overall L/W/H (+ features) with per-dimension uncertainty | `dimgram/measure.py` |
| **reconcile** | Shared dims must agree across views; flag bad captures | `dimgram/reconcile.py` |
| **draw** | ezdxf 3-view dimensioned sheet (SVG/PNG/PDF/DXF) | `dimgram/draw.py` |

---

## Quick Start

```bash
cd skills/dim-diagram/pipeline
python -m venv .venv && .venv/bin/pip install -e .

# 1. Print the board (A4 default already shipped at references/assets/charuco_A4.pdf).
#    To regenerate or resize:
.venv/bin/dimgram make-board references/assets --paper A4
#    PRINT AT 100% / ACTUAL SIZE, then verify the printed "100.0 mm" line with a ruler.

# 2. Shoot 3 photos (see references/capture-guide.md). Board BESIDE the object,
#    coplanar with the measured face: stood up for front/side, flat for top.

# 3. Measure each view.
.venv/bin/dimgram process front.jpg --view front --out front.json
.venv/bin/dimgram process top.jpg   --view top   --out top.json
.venv/bin/dimgram process side.jpg  --view side  --out side.json

# 4. Assemble the drawing + bundle.
.venv/bin/dimgram assemble front.json top.json side.json \
    --out drawing --title "My Part" --bundle ./bundle
# -> drawing.svg/.png/.pdf/.dxf  + ./bundle (for photogrammetry-3d)
```

`assemble` prints the reconciled overall dimensions and **WARNING** lines if the views disagree
(a sign one was mis-captured) or if a view's camera tilt exceeded ~2.5°.

---

## Accuracy at a Glance

Target **±1–2 %**, realistic ~0.5–1 % with care. The levers, by leverage (full detail in
`references/capture-guide.md`):

1. **Board coplanar with the measured face, beside the object** — error ≈ Δ/d (depth offset /
   camera distance). This is the single biggest lever.
2. **ChArUco board, not a bare ruler** — gives perspective correction, not just scale.
3. **Shoot far with a long lens** — distance ≥ 10× the object's depth.
4. **Square the camera** — tripod + level; the pipeline warns past ~2.5° tilt.
5. **Verify print scale** — a mis-scaled printout multiplies every dimension.

Honest limits: sub-mm needs `photogrammetry-3d` or calipers; rounded silhouettes slightly
over-measure; a single view can't see occluded features.
