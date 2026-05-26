# Design Spec: `photogrammetry-3d` — Metric 3D Model from `dim-diagram` Output + Photo Set

- **Date:** 2026-05-25
- **Status:** Approved for planning
- **Skill name (working):** `photogrammetry-3d`
- **Repo location:** `skills/photogrammetry-3d/`
- **Downstream of:** `dim-diagram` (see `2026-05-25-dim-diagram-design.md`). Data flow is
  **`dim-diagram` → `photogrammetry-3d`**: this skill **consumes `dim-diagram`'s metric
  bundle** plus an orbit/turntable photo set to build and validate a 3D model. Shares the
  ChArUco board asset and reuses `dim-diagram`'s scale, canonical axes, and silhouettes as
  priors.

---

## 1. Summary

Build a **metric 3D mesh** of an object by **fusing `dim-diagram`'s output with a multi-photo
set**. The user first runs `dim-diagram` (3 orthographic views → metric measurements, scale,
silhouettes), then feeds that **bundle** — together with a set of **overlapping photos**
(handheld orbit *or* turntable) and **other input** (camera calibration, masks) — into this
skill. **COLMAP** does Structure-from-Motion (sparse, CPU) and **OpenMVS** does the dense
reconstruction + mesh; this pairing runs on **both a CUDA Windows/Linux PC (GPU-accelerated)
and an Apple-Silicon Mac (CPU)** from one code path. The `dim-diagram` bundle supplies the two
things photogrammetry struggles with most:

- **Absolute scale** — COLMAP is scale-ambiguous; `dim-diagram`'s ChArUco-derived `mm_per_px`
  and board pose fix the metric scale directly.
- **Canonical orientation** — `dim-diagram` already labels front/top/side, so the mesh aligns
  to that frame instead of PCA-guessing.

Additionally the 3 metric **silhouettes** carve a visual hull (fill texture-poor holes, clip
strays) and the measured **L×W×H** validates the result. Outputs a scaled mesh (OBJ/PLY/STL,
best-effort STEP), a 3D measurement set, and a validation report comparing the mesh against
`dim-diagram`'s 2D measurements.

**Accuracy target:** ~0.5–2 % (realistic phone photogrammetry), reported with per-measurement
uncertainty plus the cross-check against the `dim-diagram` bundle. Metrology-grade (<0.1 %) and
reflective / featureless / transparent surfaces are out of scope.

---

## 2. Goals & non-goals

### Goals
- Reconstruct a metric 3D mesh from a photo set, **using the `dim-diagram` bundle for scale,
  canonical alignment, silhouette priors, and validation**.
- Measurements off the mesh: oriented bounding box (L×W×H), RANSAC primitive fits (planar
  faces, cylinder/hole diameters), point-to-point distances — each with uncertainty.
- **Validate** the mesh against `dim-diagram`'s 2D measurements and report discrepancies.
- Export the mesh (OBJ / PLY / STL) + measurement JSON; **best-effort STEP** for CAD.
- Support **both** capture methods (handheld orbit and turntable) with documented trade-offs.

### Non-goals
- Metrology-grade accuracy (needs coded targets + self-calibrating bundle adjustment / a rig).
- Reconstructing **reflective, transparent, or featureless** surfaces without physical surface
  prep — documented as a hard limitation, not solved in software.
- Running without a `dim-diagram` bundle. A bundle-less mode (self-recovered scale via markers)
  is a possible future fallback, **not** in scope here.
- Real-time / on-device reconstruction; photorealistic texturing beyond measurement needs.

---

## 3. User-facing workflow

```
[upstream]  dim-diagram:  3 orthographic photos ─► drawing + bundle/  (scale, silhouettes, dims)

[this skill]
  capture 20–60 overlapping photos of the SAME object:
     • handheld orbit: 2–3 rings at different heights, 60–80% overlap
     • turntable: object+markers rotate, camera fixed, ~24–36 shots × 2–3 elevations
        └─ background MUST be masked or featureless (object moves vs static background)
     (the 3 dim-diagram views may be reused as part of this set)
        │
        ▼
  photrecon reconstruct ./photos --bundle ./bundle [--masks ./masks]
        │   └─ SfM+MVS → mesh, then register to the bundle's metric frame,
        │      carve to the bundle's silhouettes
        ▼
  photrecon measure mesh.ply --bundle ./bundle   ─►  OBB L×W×H + features + validation report
  photrecon export  mesh.ply --formats obj,stl,step
```

`photrecon` is this skill's CLI. The `--bundle` directory is `dim-diagram`'s output contract
(per-view `ViewMeasurement` JSON + rectified metric images + `scale.yaml` + `camera.yaml`).

---

## 4. Architecture

Pipeline = **ingest → sfm → mvs → mesh → register → carve → measure → export**. **COLMAP does
sparse SfM (CPU, all platforms); OpenMVS does dense + mesh (CUDA on NVIDIA, CPU on Apple
Silicon)** — one dense path that runs on both a CUDA Windows/Linux box and a Mac. The
`dim-diagram` bundle drives scale, alignment, silhouette carving, and validation; Open3D /
PyMeshLab handle mesh cleanup and measurement. `photrecon.platform` detects OS/GPU and selects
the OpenMVS CUDA/CPU flag at runtime.

```
photo set ─► ingest ─► sfm ─► mvs ─► mesh ─► register ─► carve ─► measure ─► export
bundle ───────┘ │              (dense)(clean)  │            │        │       (OBJ/PLY/
(+masks)        └── intrinsics prior            │     visual hull     │        STL/STEP)
                                          scale+axes from     from bundle      + validation
                                          bundle markers      silhouettes      vs bundle dims
```

| Module | Input → Output | Key tools / notes |
|---|---|---|
| `ingest` | photo set + **bundle** + optional masks → validated inputs | Read `dim-diagram` bundle (scale, axes, silhouettes, `camera.yaml`); EXIF; **turntable masking** (segment object so static background isn't reconstructed) via `SamSegmenter` reuse or color key. |
| `sfm` | images → camera poses + sparse cloud | `pycolmap`: `extract_features` → `match_*` → `incremental_mapping`. **CPU, cross-platform** (Windows/Linux/macOS). Seed intrinsics from the bundle's `camera.yaml`. |
| `mvs` | poses + images → dense cloud + mesh | **OpenMVS** (`DensifyPointCloud` → `ReconstructMesh` → `RefineMesh` → `TextureMesh`). **Cross-platform single path**: uses **CUDA on NVIDIA** (Windows/Linux, fast), **CPU on Apple Silicon** (slower). COLMAP's own `patch_match_stereo` is *not* used (CUDA-only). `photrecon.platform.detect_compute()` picks the OpenMVS CUDA/CPU flag at runtime. |
| `mesh` | dense cloud → cleaned mesh | OpenMVS `ReconstructMesh`/`RefineMesh` output → PyMeshLab cleanup: remove floaters, decimate, fill holes. |
| `register` | mesh + bundle → **metric, canonically-aligned** mesh | Detect the **shared ChArUco markers** in the photo set, triangulate, and solve the similarity transform that maps the reconstruction onto the bundle's metric frame → applies **both scale and front/top/side axes from `dim-diagram`** (replaces self-derived scale + PCA alignment). |
| `carve` | mesh + bundle silhouettes → refined mesh | Use the 3 metric silhouettes as a **visual-hull bound**: clip strays and flag/fill texture-poor regions outside the hull. Optional but high-value for noisy reconstructions. |
| `measure` | aligned mesh + bundle → measurement set + validation | Open3D oriented bounding box → L×W×H; RANSAC plane/cylinder/sphere fits for faces & hole diameters; point-pick distances. **Cross-check vs the bundle's L×W×H + feature dims → discrepancy report.** Uncertainty from fit residuals + bundle scale σ. |
| `export` | mesh + measurements → files | OBJ/PLY/STL via trimesh/Open3D; **STEP best-effort** via FreeCAD/`pythonocc` or marked optional; measurement + validation JSON. |
| `cli` (`photrecon`) | orchestrates above | subcommands: `reconstruct` (ingest→…→carve), `measure`, `export`. |

### Shared with `dim-diagram`
- **ChArUco board asset** (dim-diagram §6) — same printed reference; markers are the
  registration anchor between the photo set and the bundle's metric frame.
- **`ViewMeasurement` + `camera.yaml` + `scale.yaml`** — consumed as the **input bundle**.
- **Accuracy philosophy** — scale verification, validation object, reported uncertainty.

*(Note: producing an enriched 2D drawing from the 3D result by feeding projections back to
`dim-diagram`'s `draw` module is an optional future enhancement, not a goal here.)*

---

## 5. Data contracts

```python
@dataclass
class DimDiagramBundle:               # the input contract from dim-diagram
    views: dict[str, ViewMeasurement] # "front"/"top"/"side": dims + sub-pixel silhouette
    rectified: dict[str, str]         # paths to metric rectified images
    mm_per_px: float                  # recovered metric scale
    board_pose: dict                  # ChArUco pose / marker layout for registration
    camera_yaml: str                  # intrinsics + distortion

@dataclass
class Reconstruction:
    cameras: list[CameraPose]
    sparse_points: np.ndarray
    mesh_path: str
    sim_transform: np.ndarray         # reconstruction → bundle metric frame (scale+R+t)

@dataclass
class Measurement3D:
    label: str
    kind: Literal["bbox", "plane", "cylinder", "distance"]
    value_mm: float | tuple           # bbox → (L, W, H)
    sigma_mm: float
    geometry: dict

@dataclass
class ValidationReport:
    pairs: list[tuple[str, float, float, float]]  # (dim, mesh_mm, bundle_mm, delta_pct)
    max_delta_pct: float
    notes: list[str]
```

---

## 6. Capture guide (`references/capture-guide.md`)

The user captures **two things for the same object**: (a) the **3 orthographic shots** for
`dim-diagram` (per its capture guide), and (b) the **overlapping photo set** for this skill.
The 3 orthographic views may be reused inside the set.

**Common:** matte/textured surfaces reconstruct well; **shiny/clear surfaces fail SfM** — dull
them (chalk spray / AESUB / powder) if acceptable, else expect holes (a real risk for the UGV
Beast's glossy parts). Even diffuse lighting, no moving shadows, no motion blur, max resolution,
keep the **ChArUco markers visible in many frames** (registration anchor), include a
known-length validation object, don't change focus/zoom mid-capture, **don't move the object**.

**Handheld orbit:** full circle at **2–3 heights**, shoot every ~10–15°, **60–80 % overlap**,
~20–60 photos. Most flexible, no rig.

**Turntable:** camera on a tripod, object + markers on the turntable, rotate in ~10–15° steps
(24–36 shots) at **2–3 camera elevations**. **Critical:** the object moves relative to a static
background, which corrupts SfM — use a **featureless/uniform background or mask the object**
(`ingest` masking). Markers ride the turntable with the object (good).

Trade-off table (rig need, background handling, consistency, effort) goes here.

---

## 7. Accuracy strategy

Ordered by leverage; lives in `references/capture-guide.md` + `measurement-3d.md`.

1. **Scale comes from the bundle (already verified in `dim-diagram`).** Because both skills use
   the same ChArUco board, the photo set registers to `dim-diagram`'s metric frame via the
   shared markers — one consistent scale, verified once with calipers upstream.
2. **Surface texture & coverage.** Texture-poor/reflective regions reconstruct as holes/noise →
   the silhouette `carve` step fills/clips them; measure where coverage is dense; report
   completeness.
3. **Overlap & view count.** More overlapping views + multiple elevations → better poses and
   denser MVS.
4. **Background contamination (turntable).** Mask or featureless background, else SfM fixes on
   the room.
5. **Lens/focus stability.** Lock focus & zoom; reuse the bundle's calibration. Motion blur
   kills matching.
6. **Validation = the cross-check.** Compare mesh OBB + feature fits against the bundle's 2D
   measurements; report `delta_pct` per dimension and the independent validation-object
   residual. This is the headline trust signal of the combined pipeline.

**Realistic accuracy:** ~0.5–2 % for careful phone capture; *not* the 1/50,000 figure that
applies only to professional coded-target rigs. Worse on shiny/featureless objects.

---

## 8. Dependencies & cross-platform support

- **Python (pip, cross-platform):** `pycolmap` (COLMAP SfM bindings), `open3d`,
  `pymeshlab`/`trimesh`, `numpy`, `opencv-contrib-python` (markers for registration), `PyYAML`.
- **System binaries (not pip):** **OpenMVS** (dense + mesh) and optionally the COLMAP CLI —
  installed via the shipped Docker images or natively (Homebrew/conda on macOS).
- **Bundle ingest:** the `dim-diagram` package (shared `ViewMeasurement`/`scale.yaml`/
  `camera.yaml` readers) or a small shared schema module.
- **Compute model (the cross-platform core):** SfM is CPU on all platforms. Dense is OpenMVS:
  **CUDA on NVIDIA** (Windows/Linux, fast), **CPU on Apple Silicon** (works, slower).
  `photrecon.platform.detect_compute()` probes OS + `nvidia-smi` and sets `openmvs_use_cuda`;
  `photrecon.platform.doctor()` prints the plan. There is **one dense code path** — only the
  CUDA/CPU flag differs — so behaviour is identical across machines, just faster on a GPU.
- **Distribution:** two Dockerfiles — `docker/Dockerfile.cuda` (`nvidia/cuda` base, OpenMVS
  built `-DOpenMVS_USE_CUDA=ON`, run with `--gpus all`) and `docker/Dockerfile.cpu`
  (Linux/CI, CUDA OFF). **macOS:** Docker can't pass through Metal, so Mac users install
  natively (brew/conda) and run OpenMVS on CPU — same backend, on the host.
- **Optional STEP:** FreeCAD / `pythonocc-core`. **Optional masking:** `ultralytics` (MobileSAM,
  which itself auto-selects CUDA/MPS/CPU — see `dim-diagram/device.py`).

---

## 9. Testing

- **Synthetic ground truth:** render a known textured mesh from many virtual cameras with
  synthetic ChArUco markers, plus a matching synthetic `dim-diagram` bundle → run the pipeline →
  assert recovered OBB within tolerance, correct registration scale, and a near-zero validation
  delta.
- **Real integration:** an object with caliper-measured truth + a real bundle → assert ±target %
  and a populated `ValidationReport`.
- **Registration:** assert the photo set aligns to the bundle frame (markers) within tolerance.
- **Turntable masking:** assert masked vs unmasked changes which geometry reconstructs.

---

## 10. Repo layout

```
skills/photogrammetry-3d/
  SKILL.md
  references/
    capture-guide.md            # orbit + turntable, surface prep, two-capture flow
    reconstruction.md           # COLMAP SfM + OpenMVS dense, masking, meshing, platform/GPU
    bundle-and-registration.md  # consuming dim-diagram bundle; marker registration; carving
    measurement-3d.md           # OBB, RANSAC primitives, validation vs bundle, export
  pipeline/
    pyproject.toml              # pip deps + [step] extra (OpenMVS/COLMAP are system binaries)
    docker/
      Dockerfile.cuda           # nvidia/cuda base; OpenMVS built with CUDA
      Dockerfile.cpu            # Linux/CI; OpenMVS CPU (macOS installs natively)
    photrecon/
      __init__.py
      platform.py               # [implemented] OS/GPU detection -> backend plan
      ingest.py                 # photo set + bundle + masks
      sfm.py                    # COLMAP (pycolmap), CPU
      mvs.py                    # OpenMVS densify+mesh (CUDA or CPU per platform.py)
      mesh.py
      register.py               # scale + canonical alignment from bundle markers
      carve.py                  # silhouette visual-hull refinement
      measure.py                # 3D measure + validation vs bundle
      export.py
      cli.py                    # photrecon
    tests/
    examples/
```

---

## 11. Implementation phasing (for the plan)

0. **[done] `platform.py`** (OS/GPU detection → backend plan) + **Dockerfiles** (CUDA + CPU).
1. **Bundle schema + `ingest`** (read `dim-diagram` bundle, photo set, masks) + CLI skeleton.
2. **`sfm`** (pycolmap poses + sparse, intrinsics from bundle) with synthetic tests.
3. **`mvs` + `mesh`** (OpenMVS densify+mesh via `platform.py` CUDA/CPU flag → PyMeshLab cleanup).
4. **`register`** (shared-marker similarity transform → metric + canonical frame from bundle).
5. **`carve`** (silhouette visual-hull refinement) — optional but high value.
6. **`measure` + validation** (OBB + RANSAC + cross-check vs bundle).
7. **`export`** (OBJ/PLY/STL + measurement/validation JSON).
8. **SKILL.md + references**, worked example (Dockerfiles already in phase 0).
9. **STEP export** (optional) + texture (optional).

MVP = phases 1–4 + 6–7 (registered metric mesh + validated measurements + export). Carving and
STEP are enhancements. **Depends on `dim-diagram` shipping its bundle output (its phases 1–5).**

---

## 12. Open questions / future

- Marker-based `register` vs COLMAP's native scale-prior — decide during planning; spec fixes
  the interface (similarity transform onto the bundle frame).
- Bundle-less fallback (self-recovered scale via markers only) — deferred.
- Neural feed-forward reconstruction (VGGT/MASt3R) as an alternative backend — deferred; COLMAP
  is the targeted engine.
- Optional `redraw`: feed 3D-derived orthographic projections back to `dim-diagram`'s `draw`
  for an enriched, fully-consistent 2D sheet — future enhancement.

---

## 13. Key references (from research)

- COLMAP / pycolmap (SfM): https://colmap.github.io/ , https://github.com/colmap/pycolmap
- OpenMVS (cross-platform dense + mesh; CUDA optional): https://github.com/cdcseacave/openMVS
- Open3D mesh + OBB + RANSAC: http://www.open3d.org/docs/release/
- PyMeshLab cleanup: https://pymeshlab.readthedocs.io/
- Visual hull / shape-from-silhouette background: https://en.wikipedia.org/wiki/Visual_hull
- Photogrammetry accuracy practices: https://www.photomodeler.com/pm-kb/getting-the-highest-possible-accuracy/
- ArUco detection (markers for registration): https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
