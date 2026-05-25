# Design Spec: `dim-diagram` — Photo → CAD-style Dimensioned Drawing

- **Date:** 2026-05-25
- **Status:** Approved for planning
- **Skill name (working):** `dim-diagram`
- **Repo location:** `skills/dim-diagram/`

---

## 1. Summary

A robotics-skills entry that turns ordinary **phone photos of an object** into a
**CAD-style, 3-view orthographic dimensioned drawing** (front / top / side), like a
manufacturer's 2D datasheet. The user captures **one photo per view**, with a
known-size **ChArUco board** (printable asset included, A4 by default) placed **beside the
object — not under it** — coplanar with the face being measured, as the scale + perspective
reference. The board's orientation changes per view (stood up vertically for front/side, laid
flat for top); see §7.1. The pipeline rectifies and measures each view independently, then
assembles the results into a dimensioned sheet exported to **SVG, PNG, and PDF** (DXF comes
free as a by-product).

The skill is delivered as **`SKILL.md` guidance + a runnable Python pipeline**, and it
**actively guides the user through correct photo capture** — capture quality, not algorithms,
dominates accuracy.

**Accuracy target:** ±1–2 % of dimension, with an honest per-dimension ± uncertainty reported
on every measurement. Sub-millimetre / metrology-grade is explicitly out of scope.

---

## 2. Goals & non-goals

### Goals
- Produce a readable 3-view orthographic drawing with linear dimensions (overall L/W/H) and
  key features (holes, slots, notable edges), plus a title block and border.
- Guide the user to capture photos that actually hit ±1–2 %.
- Ship a **print-ready A4 ChArUco board** and a generator to re-make it at other sizes.
- Report uncertainty and cross-check views against each other.
- Run on CPU with a light default dependency set; heavier ML segmentation is optional.

### Non-goals
- Full 3D reconstruction / multi-image bundle-adjustment photogrammetry (would need many
  photos; contradicts "one photo per view"). **This is handled by the separate sibling skill
  `photogrammetry-3d`** (see `2026-05-25-photogrammetry-3d-design.md`), which can hand
  orthographic projections back to this skill's `draw` renderer. The two skills **share** the
  printable ChArUco board asset (§6) and the `draw` (ezdxf) module.
- Sub-millimetre metrology (needs a controlled rig or calipers).
- Measuring occluded / internal features not visible in any single view.
- Real-time / video processing.

---

## 3. User-facing workflow

```
[one-time]  print A4 ChArUco board  →  verify printed square size with a ruler
[one-time]  calibrate phone lens    →  camera.yaml   (optional but recommended)

[per object]
  capture 3 photos (front, top, side); board beside object & coplanar with each
  measured face (stood up for front/side, flat for top — see §7.1)
        │
        ▼
  dimgram process front.jpg --view front  ─┐
  dimgram process top.jpg   --view top    ─┼─►  per-view measurement JSON
  dimgram process side.jpg  --view side   ─┘
        │
        ▼
  dimgram assemble front.json top.json side.json  ─►  drawing.{svg,png,pdf,dxf}
                                                       + consistency / uncertainty report
```

`dimgram` is the CLI. `process` handles one view; `assemble` reconciles the three and renders
the sheet. `calibrate` and `make-board` are setup utilities.

---

## 4. Architecture

Each view is an **independent 2D rectify → measure** problem — no 3D reconstruction. The
3-view layout is itself an accuracy tool (shared dimensions must agree across views).

```
                       per-view pipeline (run ×3)
  photo ─► reference ─► rectify ─► segment ─► subpixel ─► measure ─► ViewMeasurement(JSON)
            (detect    (undistort  (localize   (refine    (fit +
             ChArUco/   + homography object)    edges to   dimension)
             ArUco/      → mm/px)               sub-pixel)
             ruler)

                       assembly
  3× ViewMeasurement ─► reconcile ─► draw (ezdxf | matplotlib) ─► SVG / PNG / PDF / DXF
                        (shared-dim                              + report
                         agreement)
```

### Module responsibilities

| Module | Input → Output | Key APIs / notes |
|---|---|---|
| `calibrate` *(setup, optional)* | checkerboard/ChArUco photos → `camera.yaml` | `cv2.aruco.CharucoDetector` + `cv2.calibrateCamera` / `calibrateCameraCharuco`; target reprojection error **< 0.3 px**. Skipped ⇒ assume no distortion, warn, keep object centered. |
| `make-board` *(setup)* | size params → printable board PNG/PDF + `board.yaml` | `cv2.aruco.CharucoBoard` + `generateImage`; default **A4**; embeds a printed scale-check caption. |
| `reference` | photo → detected reference + corners | `cv2.aruco.ArucoDetector` / `CharucoDetector` (class API — **not** the removed `estimatePoseSingleMarkers`); `CORNER_REFINE_APRILTAG`. Ruler fallback = detect known length. |
| `rectify` | photo + reference + `camera.yaml` → rectified metric image, `mm_per_px`, `H`, σ | `cv2.undistort` → `cv2.findHomography`/`getPerspectiveTransform` → `cv2.warpPerspective` into a known px/mm frame. Reads marker pose to estimate tilt and **warn if > ~2–3°**. |
| `segment` | rectified image → object mask + ROI | **Swappable `Segmenter`**: `ClassicalSegmenter` (Otsu / Canny+`findContours` / `grabCut`) default; `SamSegmenter` (MobileSAM via `ultralytics`) optional. **Localization only.** |
| `subpixel` *(new)* | mask/ROI + gray image → sub-pixel contour & edge points | `cv2.cornerSubPix`; Steger/Devernay Hessian-Taylor sub-pixel contour. **This is the metrology stage** — never measure off a raw mask. |
| `measure` | sub-pixel edges → `ViewMeasurement` | Fit geometric models to sub-pixel points: `cv2.fitLine`, circle/`cv2.fitEllipse` (Hough/`minAreaRect` only **seed** these). Each `Dimension` carries `value_mm` **and `sigma_mm`**. |
| `reconcile` | 3× `ViewMeasurement` → consolidated dims + report | Reconcile shared dims (width Front↔Top, height Front↔Side, depth Top↔Side); weighted-average; flag disagreements beyond tolerance. |
| `draw` | consolidated dims → SVG/PNG/PDF/DXF | **`ezdxf` primary** (real `add_linear_dim(...).render()` / `add_diameter_dim` entities → drawing add-on backends); **matplotlib fallback** (`annotate(arrowstyle="<->")`). Third-angle layout; border + title block. |
| `cli` (`dimgram`) | orchestrates the above | subcommands: `make-board`, `calibrate`, `process`, `assemble`. |

### Design principle
Segmentation engine and rendering backend are **swappable behind interfaces**; the measurement
core never changes when you swap them. Each module is independently testable with synthetic
inputs.

---

## 5. Data contracts

```python
@dataclass
class ReferenceSpec:
    kind: Literal["charuco", "aruco", "ruler"]
    # charuco/aruco: dictionary name, board geometry; ruler: known length + endpoints
    square_length_mm: float | None
    marker_length_mm: float | None
    dictionary: str | None          # e.g. "DICT_5X5_1000"
    known_length_mm: float | None   # ruler path

@dataclass
class RectifyResult:
    image: np.ndarray               # fronto-parallel, metric
    mm_per_px: float
    homography: np.ndarray
    tilt_deg: float                 # from marker pose; drives the squareness warning
    sigma_scale: float              # fractional scale uncertainty

@dataclass
class Dimension:
    label: str
    kind: Literal["linear", "diameter", "radius"]
    p1: tuple[float, float]         # rectified-image coords (sub-pixel)
    p2: tuple[float, float]
    value_mm: float
    sigma_mm: float
    axis: Literal["X", "Y", "Z"] | None   # for cross-view reconciliation

@dataclass
class ViewMeasurement:
    view: Literal["front", "top", "side"]
    dimensions: list[Dimension]
    contour: np.ndarray             # sub-pixel
    mm_per_px: float
    warnings: list[str]

# Segmenter interface
class Segmenter(Protocol):
    def segment(self, image: np.ndarray) -> Mask: ...
```

`assemble` consumes serialized `ViewMeasurement` JSON files (NumPy arrays stored as lists).

---

## 6. ChArUco board asset (A4 default)

The skill **ships a print-ready board** so the user can start immediately, plus a generator
for other sizes.

- **Committed asset:** `references/assets/charuco_A4.pdf` and `.png`, plus `board.yaml`
  recording the exact parameters the pipeline must know.
- **Default A4 parameters** (portrait, fits with ≥15 mm margins):
  - dictionary `DICT_5X5_1000`
  - `squares_x = 7`, `squares_y = 10`
  - `square_length = 24 mm`, `marker_length = 18 mm`
  - board footprint 168 × 240 mm on 210 × 297 mm
- **Generator:** `dimgram make-board --paper A4 --square-mm 24 [--dict ...]` → recomputes a
  fit-checked board for A4/Letter/A3 and writes the asset + `board.yaml`.
- **Print-scale safety (critical):** the board image carries a **printed caption + a labelled
  100 mm calibration line** ("Print at 100 % / Actual size — this line must measure 100.0 mm;
  each square = 24.0 mm"). The user **measures it with a ruler after printing** and, if the
  printer scaled it, either reprints or passes the *measured* square size to the pipeline.
  Print scaling is a direct multiplier on every output dimension, so this check is mandatory.
- **Mounting guidance:** print on/glue to a rigid flat backing (foam board / clipboard); a
  bowed board injects corner-localization error.

---

## 7. Accuracy strategy

Captured into `references/capture-guide.md`, ordered by leverage. **Capture dominates; the
algorithm is secondary.**

### 7.1 Board placement (must be explicit and unmissable in the guide)

**The board goes *beside* the object, never under it**, lying in the *same flat plane* as the
face being measured and parallel to the camera sensor. Both the full object silhouette and the
full board must be visible and unoccluded. The board's **orientation changes per view**:

| View | Camera looks | Measured plane | Board placement |
|---|---|---|---|
| **Front** | horizontally at the front | vertical (width × height) | **stood up vertically** beside the object, facing the camera, in the front plane |
| **Side** | horizontally at the side | vertical (depth × height) | **stood up vertically** beside the object, in the side plane |
| **Top** | straight down | horizontal footprint (width × depth) | **laid flat** on the table beside the object |

Bird's-eye schematic of the **front** setup (board is a standing panel next to the object, both
at the same distance from the camera):

```
   board (standing,      object
   facing camera)        ┌──────┐
      ▓▓▓▓                │      │
   ────┴─────────────────┴──────┴────  table
                  ▲
               [phone]  far away, square-on
```

**Why not on/under the object:**
1. **Plane mismatch (dominant).** Front/side faces are *vertical*; a board lying flat is
   *horizontal* (90° off) so its homography cannot rectify that face. The board must stand in
   the face's plane.
2. **Occlusion.** An object on the board hides center squares; keep the whole board visible.
3. **Depth offset.** An object resting on the board lifts the measured silhouette above the
   board plane → error ≈ Δ/d. Side-by-side and coplanar keeps Δ ≈ 0.

**Tall-object top-view nuance:** if the widest part is elevated above the table, **raise the
flat board to that height** (prop it on a box) so it stays coplanar with the silhouette.

### Tier 1 — capture (dominates the error budget)
1. **Reference coplanar with the measured face, beside the object.** Error ≈ **Δ/d** (depth
   offset / camera distance). Place the board per §7.1 — *beside*, not under; stood up for
   front/side, flat for top; raised to the widest feature's height when that sits above the
   table.
2. **Use the ChArUco board, not a bare ruler.** 4+ corners → homography (perspective
   correction); chessboard saddle-corners localize sub-pixel; calibrates the camera for free.
3. **Shoot far with a long lens.** Distance ≥ **10× object depth**; perspective error ≈
   depth/distance.
4. **Square the camera.** Tilt error ∝ tan(tilt)·extent/d; tripod + level; pipeline warns when
   the recovered marker tilt > ~2–3°.
5. **Calibrate + undistort.** Uncorrected ≈ 1 % at frame edges; reprojection error < 0.3 px;
   keep the object off the extreme corners.

### Tier 2 — algorithm
6. **Never measure off a coarse mask.** Segment to localize → sub-pixel refine → **fit
   line/circle/ellipse** to sub-pixel points. Hough/`minAreaRect` only seed.

### Tier 3 — self-checking
7. **Cross-view reconciliation** — shared dims must agree across views; flag the bad view.
8. **Independent validation object** (known length ≥ 50 % of object size) in frame → residual
   error = honest per-shot accuracy readout.
9. **Report ± uncertainty** on every dimension.

### Error budget (goes in the guide)

| Source | If ignored | Target | Mitigation |
|---|---|---|---|
| Lens distortion | ~1 % (edges) | reproj < 0.3 px | calibrate + undistort; object off corners |
| Perspective/depth | ≈ depth/distance | distance ≥ 10× depth | far + long lens |
| **Coplanarity (Δ)** | **≈ Δ/d** | Δ < 1 % of d | board in the measured plane / mid-depth |
| Tilt/keystone | ∝ tan(tilt)·extent/d | tilt < 2–3° | tripod + level; pose warning |
| Pixel noise | sub-pixel | marker 15 px–0.5 % frame | high-res, matte, diffuse light, multi-shot |

### Honest limits (stated in the skill)
±1–2 % realistic, ~0.5–1 % with care. Sub-mm needs multi-image photogrammetry / calipers.
Rounded/3D silhouettes slightly over-measure; single views miss occluded features.

---

## 8. Phone-specific capture guide (`references/capture-guide.md`)

The guide is **phone-first** and includes:

**One-time:** print + verify the board; calibrate one specific phone lens and lock it.

**Per view (front/top/side):** matte high-contrast background → **board placed beside the
object, coplanar with the measured face per §7.1** (stood up for front/side, flat for top) →
tripod, square (use the camera-app level/grid), step back far → **main 1× or optical 2–3×
tele lens** → lock AE/AF, max resolution, **HDR/Live/Portrait/Beauty/flash OFF**, diffuse light
→ fill frame but keep clear of corners → optional validation object → 3–5 frames.

**Phone do/don't table:**

| Do | Don't | Why |
|---|---|---|
| Main 1× or optical tele | Ultra-wide 0.5× | most distorted lens |
| Move your feet | Pinch/digital zoom | upscaled crop, fake pixels |
| One calibrated lens | Switch lenses mid-session | per-lens intrinsics |
| Lock focus | Let it refocus | focus "breathing" shifts focal length |
| RAW/Pro if available | Scene-optimizer / AI enhance | computational warping moves edges |

**3-view geometry table** (which axes each view captures and how they share — width Front↔Top,
height Front↔Side, depth Top↔Side), and the **top-view caveat** (overhead mount; raise the
board to the widest feature's height).

---

## 9. Output & rendering

- **Primary renderer: `ezdxf`.** Build the three views as coordinate groups in one modelspace;
  add real dimension entities (`add_linear_dim(...).render()`, `add_diameter_dim(...).render()`
  — `.render()` is mandatory or the dim is empty); draw border/title block as LINE/TEXT.
  Export via the drawing add-on: `matplotlib.qsave()` (PNG/PDF/SVG), `SVGBackend` (SVG),
  `PyMuPdfBackend` (PDF). **DXF is the native artifact — free CAD-editable output.**
- **Fallback renderer: matplotlib.** One `Figure` → `savefig` to SVG/PDF/PNG; dimensions via
  `ax.annotate(arrowstyle="<->")` / `FancyArrowPatch`; `set_aspect('equal')`; keep figure dpi =
  savefig dpi (arrowhead-scaling gotcha).
- **No system-Cairo path** (avoids svgwrite/cairosvg dependency pain).
- **Layout:** third-angle orthographic by default (top above front, right view to the right),
  configurable; views kept in projection alignment. Title-block fields (part name, date,
  units=mm, scale, projection symbol) configurable.

---

## 10. Dependencies

- **Core (always):** `opencv-contrib-python` (provides `cv2.aruco`), `numpy`, `scipy`,
  `ezdxf`, `matplotlib`, `PyYAML`. `imutils` optional convenience.
- **Optional SAM backend:** `ultralytics` (pulls MobileSAM ~40 MB weights). Behind an extras
  group; import guarded so the core runs without torch.
- **Optional PDF backend:** `PyMuPDF` (if not using the matplotlib PDF path).

---

## 11. Testing

- **Synthetic ground truth (unit):** programmatically render rectangles + circles with a
  synthetic ChArUco/ArUco at known scale and known perspective; assert `rectify`/`measure`/
  `reconcile` recover dims within tolerance. Deterministic, no fixtures.
- **Real integration:** a small fixture set of an object with caliper-measured truth → assert
  ±1–2 %; assert the validation-object residual is reported.
- **Reconcile logic:** feed inconsistent views → assert the bad view is flagged.
- **Renderer golden:** snapshot SVG structure / element counts; assert all of SVG/PNG/PDF are
  produced.
- **Board generator:** assert the A4 board fits with margins and `board.yaml` round-trips.

---

## 12. Repo layout

```
skills/dim-diagram/
  SKILL.md
  references/
    capture-guide.md          # phone-first, Tier 1–3, error budget, do/don't, 3-view geometry
    rectification.md          # ChArUco/ArUco/ruler, ArucoDetector API, homography, calibration
    measurement.md            # segmentation, sub-pixel refinement, model fitting, features
    drawing-output.md         # ezdxf dims, layout, title block, formats
    assets/
      charuco_A4.pdf
      charuco_A4.png
      board.yaml
  pipeline/
    pyproject.toml            # core deps + [sam] extra
    dimgram/
      __init__.py
      reference.py
      rectify.py
      segment.py              # Segmenter, ClassicalSegmenter, SamSegmenter
      subpixel.py
      measure.py
      reconcile.py
      draw.py                 # EzdxfRenderer, MatplotlibRenderer
      board.py                # make-board
      calibrate.py
      cli.py                  # dimgram
    tests/
    examples/
```

`SKILL.md` follows the repo convention (cv-detection style): frontmatter `name` + rich
`description` with trigger keywords, a quick-nav block to the four references, a who-it's-for
intro, and a worked end-to-end example.

---

## 13. Implementation phasing (for the plan)

1. **Skeleton + data contracts + CLI stub** (`process`/`assemble` wiring, JSON I/O).
2. **`make-board` + committed A4 asset** + print-scale caption.
3. **`reference` + `rectify`** (ChArUco/ArUco homography; ruler fallback; tilt warning) with
   synthetic tests.
4. **`segment` (classical) + `subpixel` + `measure`** → single-view `ViewMeasurement` with
   synthetic-truth accuracy test.
5. **`reconcile` + `draw` (ezdxf)** → full 3-view sheet (SVG/PNG/PDF/DXF) + report.
6. **`calibrate`** utility + undistortion wired into `rectify`.
7. **`SamSegmenter` optional backend** behind the `[sam]` extra.
8. **SKILL.md + four references** (incl. the phone capture guide) + worked example.
9. **matplotlib fallback renderer** (if time / for the no-ezdxf path).

MVP = phases 1–5 + 8 (classical, ChArUco, ezdxf). SAM, calibration, and matplotlib fallback
are enhancements.

---

## 14. Open questions / future

- Auto-classify which photo is which view (vs. requiring `--view`)? Defer; require the flag.
- VLM-assisted feature labelling ("which dims matter") — possible later enhancement.
- Letter/A3 board presets shipped as assets vs. generated on demand — generate on demand for
  now, ship only A4.

---

## 15. Key references (from research)

- OpenCV ArUco detection (class API; `estimatePoseSingleMarkers` deprecated):
  https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- ChArUco calibration: https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
- PyImageSearch size-measurement (pixels-per-metric baseline):
  https://pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
- MobileSAM (CPU-light SAM): https://docs.ultralytics.com/models/mobile-sam
- Sub-pixel edge contour (Steger): https://github.com/raymondngiam/subpixel-edge-contour-in-opencv
- ezdxf drawing add-on + dims: https://ezdxf.readthedocs.io/en/stable/addons/drawing.html ,
  https://ezdxf.readthedocs.io/en/stable/tutorials/linear_dimension.html
- Single-view metrology error analysis (Criminisi):
  https://www.robots.ox.ac.uk/~vgg/publications/1999/Criminisi99b/criminisi99b.pdf
- Photogrammetry accuracy practices: https://www.photomodeler.com/pm-kb/getting-the-highest-possible-accuracy/
- First vs third angle: https://www.gdandtbasics.com/first-vs-third-angle-orthographic-views/
```
