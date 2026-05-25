---
name: photogrammetry-3d
description: >
  Reconstruct a metric 3D mesh of an object from many overlapping phone photos using COLMAP
  Structure-from-Motion + Multi-View Stereo, scaled and aligned from a ChArUco board. Use when
  the user wants a 3D model / mesh / point cloud from photos, photogrammetry, COLMAP/SfM/MVS,
  scan a real object into OBJ/STL/PLY/STEP, measure a 3D object's oriented bounding box or hole
  diameters from a photo set, or turn a turntable/orbit capture into a CAD model. Downstream of
  the `dim-diagram` skill: it consumes dim-diagram's metric bundle (scale, canonical axes,
  silhouettes) plus the photo set to build and validate the model. Needs MANY overlapping photos;
  for a single photo (or 3 orthographic shots) and a flat 2D dimensioned drawing, use dim-diagram
  instead. Not for mesh file-format conversion (e.g. STL->OBJ), panorama stitching, SLAM/visual
  odometry, or NeRF / gaussian-splat view synthesis.
---

# Phone Photos → Metric 3D Mesh (`photogrammetry-3d`)

> **Quick navigation**
> - How to capture (orbit + turntable, surface prep) -> `references/capture-guide.md`
> - COLMAP SfM/MVS, masking, meshing -> `references/reconstruction-colmap.md`
> - Consuming the dim-diagram bundle; marker registration; silhouette carving -> `references/bundle-and-registration.md`
> - 3D measurement (OBB, RANSAC), validation vs bundle, export -> `references/measurement-3d.md`

> **Status:** Design + guidance skill. The runnable pipeline (`photrecon`) is specced in
> `docs/superpowers/specs/2026-05-25-photogrammetry-3d-design.md` and not yet implemented; this
> SKILL.md captures the method and the interfaces so the work can proceed against a fixed design.

---

## Who This Skill Is For

This skill is for getting a **metric 3D mesh** of an object from a phone — the heavy,
high-fidelity counterpart to `dim-diagram`. Use it when a flat 3-view drawing isn't enough:
complex/organic geometry, many features, or when you want the **mesh / CAD model** itself
(OBJ/STL/PLY, best-effort STEP).

The crucial design choice: **this skill is downstream of `dim-diagram`.** Photogrammetry's two
hardest problems — absolute scale and canonical orientation — are already solved by
`dim-diagram`'s ChArUco-based output. So you run `dim-diagram` first (3 orthographic photos →
metric `bundle/`), then feed that bundle **plus** an orbit/turntable photo set into this skill.

It is **not** metrology-grade (~0.5–2 % realistic for phone photogrammetry), and it **fails on
reflective/transparent/featureless surfaces** without physical surface prep.

## Pipeline Architecture

```
photo set ─► ingest ─► sfm ─► mvs ─► mesh ─► register ─► carve ─► measure ─► export
bundle ───────┘ │              (dense)(clean)  │            │        │       (OBJ/PLY/
(+masks)        └── intrinsics prior            │     visual hull     │        STL/STEP)
                                          scale+axes from     from bundle      + validation
                                          bundle markers      silhouettes      vs bundle dims
```

| Stage | What happens |
|-------|--------------|
| **ingest** | Load photo set + dim-diagram bundle (+ turntable masks) |
| **sfm** | `pycolmap` feature extraction/matching/mapping → camera poses + sparse cloud |
| **mvs** | COLMAP `patch_match_stereo` + `stereo_fusion` → dense cloud (GPU recommended) |
| **mesh** | Poisson meshing + PyMeshLab cleanup (floaters, decimate, fill holes) |
| **register** | Shared ChArUco markers → similarity transform onto the bundle's **metric, canonical** frame |
| **carve** | Bundle silhouettes as a visual-hull bound — clip strays, fill texture-poor holes |
| **measure** | Oriented bounding box (L×W×H) + RANSAC plane/cylinder fits; **cross-check vs bundle dims** |
| **export** | OBJ/PLY/STL + best-effort STEP + measurement & validation JSON |

## Quick Start (intended CLI; pipeline not yet implemented)

```bash
# 0. Run dim-diagram first to get ./bundle (scale + silhouettes + dims).
# 1. Capture 20-60 overlapping photos (see references/capture-guide.md).
photrecon reconstruct ./photos --bundle ./bundle [--masks ./masks]   # -> scaled mesh
photrecon measure mesh.ply --bundle ./bundle                          # -> OBB + validation
photrecon export  mesh.ply --formats obj,stl,step
```

## Accuracy at a Glance

1. **Scale comes from the bundle** (already verified with calipers upstream in `dim-diagram`) —
   the photo set registers to that metric frame via the shared ChArUco markers.
2. **Surface texture & coverage** — shiny/featureless regions reconstruct as holes; the
   silhouette `carve` step fills/clips them; measure where coverage is dense.
3. **Overlap & view count** — 60–80 % overlap, multiple elevations.
4. **Background contamination (turntable)** — mask the object or use a featureless backdrop.
5. **Validation is the cross-check** — mesh OBB vs the bundle's 2D dims, reported as `delta_pct`.

Realistic ~0.5–2 %; sub-mm needs a coded-target rig or calipers.
