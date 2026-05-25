# Design Spec: `photogrammetry-3d` — Phone Photos → Metric 3D Mesh + Measurements

- **Date:** 2026-05-25
- **Status:** Approved for planning
- **Skill name (working):** `photogrammetry-3d`
- **Repo location:** `skills/photogrammetry-3d/`
- **Sibling of:** `dim-diagram` (see `2026-05-25-dim-diagram-design.md`) — shares the ChArUco
  board asset and the `draw` (ezdxf) renderer; can hand orthographic projections back to it.

---

## 1. Summary

Reconstruct a **metric 3D mesh** of an object from **many overlapping phone photos** (handheld
orbit *or* turntable), using **COLMAP** Structure-from-Motion + Multi-View Stereo. Absolute
scale comes from **known-size ChArUco/ArUco markers** in the scene (the same printable board as
`dim-diagram`). Outputs a scaled mesh (OBJ/PLY/STL), a measurement set (oriented bounding-box
L×W×H + fitted features), and — for the 2D datasheet use case — **canonical orthographic
projections handed off to `dim-diagram`'s renderer**.

This is the heavy, high-fidelity counterpart to `dim-diagram`: complex geometry, full 3D, and a
mesh deliverable, at the cost of many photos, heavier dependencies, and a GPU.

**Accuracy target:** ~0.5–2 % (realistic phone photogrammetry), reported with per-measurement
uncertainty and a scale-bar/validation residual. Metrology-grade (<0.1 %) and reflective /
featureless / transparent surfaces are explicitly out of scope.

---

## 2. Goals & non-goals

### Goals
- Metric 3D mesh from a phone photo set, scaled from known-size markers.
- Measurements off the mesh: oriented bounding box (L×W×H), RANSAC primitive fits (planar
  faces, cylinder/hole diameters), point-to-point distances — each with uncertainty.
- Export the mesh (OBJ / PLY / STL) and a measurement JSON; **best-effort STEP** for CAD.
- Produce canonical front/top/side (and isometric) orthographic projections and **hand them to
  `dim-diagram`'s `draw` module** to make the 2D dimensioned sheet.
- Support **both** capture methods (handheld orbit and turntable) with documented trade-offs.

### Non-goals
- Metrology-grade accuracy (needs coded targets + self-calibrating bundle adjustment / a rig).
- Reconstructing **reflective, transparent, or featureless** surfaces without physical surface
  prep — documented as a hard limitation, not solved in software.
- Real-time / on-device reconstruction.
- Photorealistic texturing beyond what measurement and projection need (a textured mesh is a
  nice-to-have, not a goal).

---

## 3. User-facing workflow

```
[one-time]  print the shared A4 ChArUco board (from dim-diagram §6); verify printed size
[one-time]  (optional) calibrate the phone lens

[per object]
  capture 20–60 overlapping photos:
     • handheld orbit: 2–3 rings at different heights, 60–80% overlap
     • turntable: object+markers rotate, camera fixed, ~24–36 shots × 2–3 elevations
        └─ background MUST be masked or featureless (object moves vs static background)
        │
        ▼
  photrecon reconstruct ./photos --markers charuco  ─►  scaled mesh + camera poses
  photrecon measure mesh.ply                        ─►  OBB L×W×H + features (JSON)
  photrecon export  mesh.ply --formats obj,stl,step ─►  mesh files
  photrecon project mesh.ply --views front,top,side ─►  per-view contour JSON
        │
        ▼
  dimgram assemble front.json top.json side.json    ─►  drawing.{svg,png,pdf,dxf}
```

`photrecon` is this skill's CLI; `dimgram` is the sibling's. The `project` output uses the same
`ViewMeasurement` JSON contract `dim-diagram`'s `assemble` consumes — that is the handoff.

---

## 4. Architecture

Pipeline = **reconstruct → scale → align → measure → (export | project)**. COLMAP does the
heavy reconstruction; Open3D / PyMeshLab handle mesh ops and measurement.

```
photos ─► ingest ─► sfm ─► mvs ─► mesh ─► scale ─► align ─► measure ─► export
         (+masks)  (poses) (dense) (clean) (metric) (canon.  (OBB,      (OBJ/PLY/
                                                     frame)   features)  STL/STEP)
                                                                │
                                                                └─► project ─► ViewMeasurement
                                                                    (ortho)     JSON → dim-diagram
```

| Module | Input → Output | Key tools / notes |
|---|---|---|
| `ingest` | image dir → validated set (+ optional masks) | EXIF read; **turntable masking** (segment object so static background isn't reconstructed) via `SamSegmenter` reuse or color key. |
| `sfm` | images → camera poses + sparse cloud | `pycolmap`: `extract_features` → `match_*` → `incremental_mapping`. Exhaustive matching for small sets, vocab-tree/sequential for larger. |
| `mvs` | poses + images → dense point cloud | COLMAP `patch_match_stereo` + `stereo_fusion`. **GPU strongly recommended.** |
| `mesh` | dense cloud → cleaned mesh | Poisson (`poisson_mesher` / Open3D `create_from_point_cloud_poisson`) → PyMeshLab cleanup: remove floaters, decimate, fill holes. |
| `scale` | reconstruction + marker images → metric scale factor | Detect ArUco/ChArUco corners (`cv2.aruco`), triangulate into the reconstruction, solve scale from known marker/board size; apply to mesh + poses. **Direct multiplier on every dimension — the #1 accuracy lever.** |
| `align` | mesh → canonical-axis mesh + OBB | PCA / user-specified up+front face → axis-aligned frame so projections are meaningful. |
| `measure` | aligned mesh → measurement set | Open3D oriented bounding box → L×W×H; RANSAC plane/cylinder/sphere fits (Open3D `segment_plane`, PyMeshLab/`pyransac3d`) for faces & hole diameters; point-pick distances. Uncertainty from fit residuals + scale σ. |
| `export` | mesh + measurements → files | OBJ/PLY/STL via trimesh/Open3D; **STEP best-effort** via FreeCAD/`pythonocc` or marked optional; measurement JSON. |
| `project` | aligned mesh → per-view contours | Orthographic silhouette + visible-edge extraction along canonical axes → `ViewMeasurement` JSON (dim-diagram contract). Optional isometric. |
| `cli` (`photrecon`) | orchestrates above | subcommands: `reconstruct` (ingest→mesh→scale→align), `measure`, `export`, `project`. |

### Shared with `dim-diagram`
- **ChArUco board asset** (§6 of the dim-diagram spec) — same printed reference.
- **`draw` (ezdxf) renderer** — reused via the `ViewMeasurement` handoff; this skill does not
  re-implement drawing.
- **Accuracy philosophy** — scale verification, validation object, reported uncertainty.

---

## 5. Data contracts

```python
@dataclass
class Reconstruction:
    cameras: list[CameraPose]        # COLMAP poses
    sparse_points: np.ndarray
    dense_mesh_path: str
    metric_scale: float              # applied factor (mm per reconstruction-unit)
    scale_sigma: float

@dataclass
class Measurement3D:
    label: str
    kind: Literal["bbox", "plane", "cylinder", "distance"]
    value_mm: float | tuple          # bbox → (L, W, H)
    sigma_mm: float
    geometry: dict                   # fit params / endpoints

# project() emits dim-diagram's existing ViewMeasurement JSON — no new drawing contract.
```

---

## 6. Capture guide (`references/capture-guide.md`) — both methods

**Common:** matte/textured surfaces reconstruct well; **shiny/clear surfaces fail SfM** — dull
them (chalk spray / AESUB / powder) if acceptable, else expect holes (a real risk for the UGV
Beast's glossy parts). Even diffuse lighting, no moving shadows, no motion blur, max resolution,
keep markers of known size visible in many frames, include a known-length validation object,
don't change focus/zoom mid-capture.

**Handheld orbit:** walk a full circle at **2–3 heights** (low/eye/high), shoot every ~10–15°,
keep **60–80 % overlap**, ~20–60 photos, object + markers fully in frame, **don't move the
object**. Most flexible, no rig.

**Turntable:** camera on a tripod, object + markers on the turntable, rotate in ~10–15° steps
(24–36 shots) at **2–3 camera elevations**. **Critical:** the object moves relative to a static
background, which corrupts SfM — use a **featureless/uniform background or mask the object**
(`ingest` masking). Markers ride the turntable with the object (good). More consistent lighting;
needs background handling.

Trade-off summary table (rig need, background handling, consistency, effort) goes here.

---

## 7. Accuracy strategy

Ordered by leverage; lives in `references/capture-guide.md` + `measurement-3d.md`.

1. **Scale-bar accuracy = direct multiplier.** Verify the printed marker size with calipers;
   use markers/scale ≥ ~50 % of object size; multiple scales reduce error. Any % error in the
   known size is a 1:1 % error in every output dimension.
2. **Surface texture & coverage.** Texture-poor or reflective regions reconstruct as
   holes/noise → measure where coverage is dense; report completeness.
3. **Overlap & view count.** More overlapping views + multiple elevations → better poses and
   denser MVS. Sparse capture → drift.
4. **Background contamination (turntable).** Mask or featureless background, else the
   reconstruction fixes on the room, not the object.
5. **Lens / focus stability.** Lock focus & zoom; optional calibration. Motion blur kills
   feature matching.
6. **Report:** scale residual, COLMAP reprojection error, dense point count / mesh
   completeness, OBB and feature-fit σ, and the **independent validation-object residual**.

**Realistic accuracy:** ~0.5–2 % for careful phone capture; *not* the 1/50,000 figure that
applies only to professional coded-target rigs. Worse on shiny/featureless objects.

---

## 8. Dependencies

- **Core:** `pycolmap` (or the COLMAP binary), `open3d`, `pymeshlab`/`trimesh`, `numpy`,
  `opencv-contrib-python` (markers), `PyYAML`.
- **Rendering handoff:** reuse `dim-diagram`'s `ezdxf` path (dependency on the sibling package
  or a shared module).
- **GPU strongly recommended** (CUDA) for `patch_match_stereo`; CPU works but is slow.
- **Optional STEP:** FreeCAD / `pythonocc-core`.
- **Optional masking backend:** `ultralytics` (MobileSAM), shared concept with dim-diagram.
- **Distribution:** ship a **Dockerfile** (COLMAP + CUDA) given the heavy native deps.

---

## 9. Testing

- **Synthetic ground truth:** render a known textured mesh from many virtual cameras with
  synthetic markers of known size → run the pipeline → assert recovered OBB within tolerance and
  scale factor correct. Deterministic where COLMAP allows; otherwise fixture-based.
- **Real integration:** an object with caliper-measured truth → assert ±target %; assert the
  validation-object residual is reported.
- **Handoff:** assert `project` output validates against `dim-diagram`'s `ViewMeasurement`
  schema and renders.
- **Turntable masking:** assert masked vs unmasked changes which geometry reconstructs.

---

## 10. Repo layout

```
skills/photogrammetry-3d/
  SKILL.md
  references/
    capture-guide.md            # orbit + turntable, surface prep, scale, validation
    reconstruction-colmap.md    # pycolmap SfM/MVS, masking, meshing, scale recovery
    measurement-3d.md           # OBB, RANSAC primitives, uncertainty
    handoff-and-export.md       # ViewMeasurement handoff to dim-diagram; OBJ/STL/STEP
  pipeline/
    pyproject.toml              # core deps + [step]/[sam] extras
    Dockerfile                  # COLMAP + CUDA
    photrecon/
      __init__.py
      ingest.py
      sfm.py
      mvs.py
      mesh.py
      scale.py
      align.py
      measure.py
      project.py
      export.py
      cli.py                    # photrecon
    tests/
    examples/
```

---

## 11. Implementation phasing (for the plan)

1. **Capture guide + `ingest`** (image validation, EXIF, turntable masking) + CLI skeleton.
2. **`sfm`** (pycolmap poses + sparse) with synthetic-camera tests.
3. **`mvs` + `mesh`** (dense + Poisson + cleanup).
4. **`scale`** (marker triangulation → metric) + **`align`** (canonical OBB frame).
5. **`measure`** (OBB + RANSAC primitives + uncertainty).
6. **`export`** (OBJ/PLY/STL + measurement JSON).
7. **`project` → handoff** to `dim-diagram` renderer.
8. **SKILL.md + references**, Dockerfile, worked example.
9. **STEP export** (optional) + texture (optional).

MVP = phases 1–6 (scaled mesh + measurements). Projection/handoff and STEP are enhancements.

---

## 12. Open questions / future

- Use COLMAP's native scale-prior support vs. post-hoc marker triangulation for scale — decide
  during planning; spec fixes only the interface.
- Neural feed-forward reconstruction (VGGT/MASt3R) as an *alternative* fast backend behind the
  same interface — deferred; COLMAP is the targeted engine.
- Auto canonical-frame detection vs. requiring a user-specified front/up — start with
  PCA + a manual override flag.

---

## 13. Key references (from research)

- COLMAP / pycolmap: https://colmap.github.io/ , https://github.com/colmap/pycolmap
- Open3D mesh + OBB + RANSAC: http://www.open3d.org/docs/release/
- PyMeshLab cleanup: https://pymeshlab.readthedocs.io/
- Photogrammetry accuracy practices: https://www.photomodeler.com/pm-kb/getting-the-highest-possible-accuracy/
- Multi-view calibration & accuracy: https://daetechsystems.com/why-multi-view-calibration-makes-photogrammetry-so-accurate/
- ArUco detection (markers for scale): https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
```
