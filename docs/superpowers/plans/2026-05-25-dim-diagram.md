# dim-diagram MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the `dim-diagram` skill MVP: turn three orthographic phone photos (front/top/side), each with a ChArUco board for scale, into a CAD-style dimensioned drawing (SVG/PNG/PDF/DXF) plus a metric `bundle/` for the downstream `photogrammetry-3d` skill.

**Architecture:** A Python package `dimgram` with a per-view pipeline (`reference → rectify → segment → subpixel → measure`) that runs independently on each photo, then `reconcile` (cross-view shared-dimension agreement) and `draw` (ezdxf renderer). A `dimgram` CLI orchestrates `make-board`, `process` (one view), and `assemble` (three views → drawing + bundle). ChArUco detection gives metric scale + perspective rectification via homography; classical OpenCV does segmentation; sub-pixel refinement + model fitting produce the numbers.

**Tech Stack:** Python 3.11+, `opencv-contrib-python` (cv2.aruco), `numpy`, `scipy`, `ezdxf` (+ matplotlib backend), `PyYAML`, `pytest`.

**Scope:** This plan covers spec phases 1–5 + 8 (the MVP boundary). Deferred to a follow-on plan: `SamSegmenter` ([sam] extra), `calibrate`/undistortion, and the matplotlib fallback renderer. Spec: `docs/superpowers/specs/2026-05-25-dim-diagram-design.md`.

---

## File Structure

```
skills/dim-diagram/
  SKILL.md                          # Task 13
  references/
    capture-guide.md                # Task 13
    rectification.md                # Task 13
    measurement.md                  # Task 13
    drawing-output.md               # Task 13
    assets/{charuco_A4.png,.pdf,board.yaml}   # produced by Task 3
  pipeline/
    pyproject.toml                  # Task 1
    dimgram/
      __init__.py                   # Task 1
      types.py                      # Task 2  (data contracts)
      board.py                      # Task 3  (make-board)
      reference.py                  # Task 4  (ChArUco detection)
      rectify.py                    # Task 5  (homography → metric image)
      segment.py                    # Task 6  (classical segmentation)
      subpixel.py                   # Task 7  (sub-pixel contour)
      measure.py                    # Task 8  (dimensions + ViewMeasurement)
      reconcile.py                  # Task 9  (cross-view agreement)
      draw.py                       # Task 10 (ezdxf renderer)
      bundle.py                     # Task 11 (bundle export)
      cli.py                        # Task 12 (dimgram CLI)
    tests/
      conftest.py                   # Task 2  (synthetic fixtures)
      test_types.py                 # Task 2
      test_board.py                 # Task 3
      test_reference.py             # Task 4
      test_rectify.py               # Task 5
      test_segment.py               # Task 6
      test_subpixel.py              # Task 7
      test_measure.py               # Task 8
      test_reconcile.py             # Task 9
      test_draw.py                  # Task 10
      test_bundle.py                # Task 11
      test_cli.py                   # Task 12
```

All commands below run from `skills/dim-diagram/pipeline/` unless stated otherwise.

---

### Task 1: Project scaffold

**Files:**
- Create: `skills/dim-diagram/pipeline/pyproject.toml`
- Create: `skills/dim-diagram/pipeline/dimgram/__init__.py`
- Create: `skills/dim-diagram/pipeline/tests/__init__.py`

- [ ] **Step 1: Create `pyproject.toml`**

```toml
[project]
name = "dimgram"
version = "0.1.0"
description = "Photo -> CAD-style dimensioned drawing pipeline"
requires-python = ">=3.11"
dependencies = [
    "opencv-contrib-python>=4.10",
    "numpy>=1.26",
    "scipy>=1.11",
    "ezdxf>=1.3",
    "matplotlib>=3.8",
    "PyYAML>=6.0",
]

[project.optional-dependencies]
dev = ["pytest>=8.0"]
sam = ["ultralytics>=8.2"]

[project.scripts]
dimgram = "dimgram.cli:main"

[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
where = ["."]
include = ["dimgram*"]

[tool.pytest.ini_options]
testpaths = ["tests"]
```

- [ ] **Step 2: Create empty package files**

`dimgram/__init__.py`:
```python
"""dim-diagram: photo -> CAD-style dimensioned drawing."""
__version__ = "0.1.0"
```

`tests/__init__.py`: (empty file)

- [ ] **Step 3: Install editable and verify import**

Run: `pip install -e ".[dev]"`
Then: `python -c "import dimgram, cv2, ezdxf; print(dimgram.__version__, cv2.__version__)"`
Expected: prints `0.1.0` and an OpenCV version `>= 4.10`. Confirm `cv2.aruco` exists: `python -c "import cv2; cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)"` (no error).

- [ ] **Step 4: Commit**

```bash
git add skills/dim-diagram/pipeline/pyproject.toml skills/dim-diagram/pipeline/dimgram/__init__.py skills/dim-diagram/pipeline/tests/__init__.py
git commit -m "feat(dim-diagram): project scaffold and dependencies"
```

---

### Task 2: Data contracts + synthetic test fixtures

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/types.py`
- Create: `skills/dim-diagram/pipeline/tests/conftest.py`
- Test: `skills/dim-diagram/pipeline/tests/test_types.py`

- [ ] **Step 1: Write the failing test**

`tests/test_types.py`:
```python
import numpy as np
from dimgram.types import Dimension, ViewMeasurement, to_json, from_json

def test_dimension_roundtrip(tmp_path):
    vm = ViewMeasurement(
        view="front",
        dimensions=[Dimension(label="width", kind="linear",
                              p1=(0.0, 0.0), p2=(100.0, 0.0),
                              value_mm=100.0, sigma_mm=0.5, axis="X")],
        contour=np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 50.0]]),
        mm_per_px=0.5,
        warnings=["tilt 1.2 deg"],
    )
    path = tmp_path / "front.json"
    to_json(vm, path)
    loaded = from_json(path)
    assert loaded.view == "front"
    assert loaded.dimensions[0].value_mm == 100.0
    assert loaded.dimensions[0].axis == "X"
    assert loaded.mm_per_px == 0.5
    np.testing.assert_allclose(loaded.contour, vm.contour)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_types.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.types'`.

- [ ] **Step 3: Implement `types.py`**

```python
from __future__ import annotations
import json
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import Literal, Optional
import numpy as np

Axis = Literal["X", "Y", "Z"]
View = Literal["front", "top", "side"]

@dataclass
class Dimension:
    label: str
    kind: Literal["linear", "diameter", "radius"]
    p1: tuple[float, float]
    p2: tuple[float, float]
    value_mm: float
    sigma_mm: float
    axis: Optional[Axis] = None

@dataclass
class ViewMeasurement:
    view: View
    dimensions: list[Dimension]
    contour: np.ndarray            # (N, 2) float, rectified-image px (sub-pixel)
    mm_per_px: float
    warnings: list[str] = field(default_factory=list)

def to_json(vm: ViewMeasurement, path: Path) -> None:
    payload = {
        "view": vm.view,
        "dimensions": [asdict(d) for d in vm.dimensions],
        "contour": np.asarray(vm.contour, dtype=float).tolist(),
        "mm_per_px": vm.mm_per_px,
        "warnings": list(vm.warnings),
    }
    Path(path).write_text(json.dumps(payload, indent=2))

def from_json(path: Path) -> ViewMeasurement:
    payload = json.loads(Path(path).read_text())
    dims = [Dimension(**{**d, "p1": tuple(d["p1"]), "p2": tuple(d["p2"])})
            for d in payload["dimensions"]]
    return ViewMeasurement(
        view=payload["view"],
        dimensions=dims,
        contour=np.array(payload["contour"], dtype=float),
        mm_per_px=payload["mm_per_px"],
        warnings=payload.get("warnings", []),
    )
```

- [ ] **Step 4: Add the synthetic-scene fixture in `conftest.py`**

This fixture renders a ChArUco board into a blank scene under a known homography, plus a known-size rectangle, so multiple later tasks can drive TDD against ground truth.

```python
import cv2
import numpy as np
import pytest

# Board geometry shared across tests (matches the A4 default in board.py).
DICT = cv2.aruco.DICT_5X5_1000
SQUARES_X, SQUARES_Y = 7, 10
SQUARE_MM, MARKER_MM = 24.0, 18.0

@pytest.fixture
def board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_MM, MARKER_MM, d)

@pytest.fixture
def synthetic_scene(board):
    """A fronto-parallel scene at a known scale: board on the left, a known
    100 x 60 mm rectangle on the right. Returns (image, mm_per_px, rect_mm)."""
    mm_per_px = 0.25                      # 4 px per mm
    px = lambda mm: int(round(mm / mm_per_px))
    board_w_mm, board_h_mm = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    margin = px(20)
    rect_mm = (100.0, 60.0)
    canvas_h = margin * 2 + px(board_h_mm)
    canvas_w = margin * 3 + px(board_w_mm) + px(rect_mm[0])
    img = np.full((canvas_h, canvas_w), 255, np.uint8)

    board_img = board.generateImage((px(board_w_mm), px(board_h_mm)))
    img[margin:margin + px(board_h_mm), margin:margin + px(board_w_mm)] = board_img

    rx0 = margin * 2 + px(board_w_mm)
    ry0 = margin
    cv2.rectangle(img, (rx0, ry0), (rx0 + px(rect_mm[0]), ry0 + px(rect_mm[1])), 0, -1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), mm_per_px, rect_mm
```

- [ ] **Step 5: Run tests and commit**

Run: `pytest tests/test_types.py -v`
Expected: PASS.
```bash
git add skills/dim-diagram/pipeline/dimgram/types.py skills/dim-diagram/pipeline/tests/conftest.py skills/dim-diagram/pipeline/tests/test_types.py
git commit -m "feat(dim-diagram): data contracts + synthetic scene fixture"
```

---

### Task 3: `make-board` — printable A4 ChArUco board

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/board.py`
- Test: `skills/dim-diagram/pipeline/tests/test_board.py`

- [ ] **Step 1: Write the failing test**

```python
import yaml
from dimgram.board import make_board, A4_PRESET

def test_make_board_fits_a4(tmp_path):
    out = make_board(tmp_path, preset="A4")
    cfg = yaml.safe_load((tmp_path / "board.yaml").read_text())
    # footprint must fit on A4 (210 x 297 mm) with >= 15 mm margins
    assert cfg["squares_x"] * cfg["square_mm"] <= 210 - 30
    assert cfg["squares_y"] * cfg["square_mm"] <= 297 - 30
    assert (tmp_path / "charuco_A4.png").exists()
    assert out["board_w_mm"] == cfg["squares_x"] * cfg["square_mm"]
    assert cfg["dictionary"] == "DICT_5X5_1000"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_board.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.board'`.

- [ ] **Step 3: Implement `board.py`**

```python
from __future__ import annotations
from pathlib import Path
import cv2
import numpy as np
import yaml

PAPER_MM = {"A4": (210, 297), "Letter": (216, 279), "A3": (297, 420)}
A4_PRESET = dict(dictionary="DICT_5X5_1000", squares_x=7, squares_y=10,
                 square_mm=24.0, marker_mm=18.0)
PRINT_DPI = 300

def _mm_to_px(mm: float, dpi: int = PRINT_DPI) -> int:
    return int(round(mm / 25.4 * dpi))

def make_board(out_dir: Path, preset: str = "A4", square_mm: float | None = None,
               dictionary: str = "DICT_5X5_1000") -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paper_w, paper_h = PAPER_MM[preset]
    sq = square_mm or A4_PRESET["square_mm"]
    marker_mm = round(sq * 0.75, 1)
    squares_x = int((paper_w - 30) // sq)        # >=15mm margins each side
    squares_y = int((paper_h - 30) // sq)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    bd = cv2.aruco.CharucoBoard((squares_x, squares_y), sq, marker_mm, d)
    board_w_mm, board_h_mm = squares_x * sq, squares_y * sq
    img = bd.generateImage((_mm_to_px(board_w_mm), _mm_to_px(board_h_mm)), marginSize=0)

    # white A4 page with the board centred + a printed 100 mm scale-check line
    page = np.full((_mm_to_px(paper_h), _mm_to_px(paper_w)), 255, np.uint8)
    ox, oy = _mm_to_px((paper_w - board_w_mm) / 2), _mm_to_px((paper_h - board_h_mm) / 2)
    page[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
    y = oy + img.shape[0] + _mm_to_px(8)
    cv2.line(page, (ox, y), (ox + _mm_to_px(100), y), 0, 2)
    cv2.putText(page, f"Print at 100% / Actual size. This line = 100.0 mm; square = {sq} mm",
                (ox, y + _mm_to_px(6)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)

    png = out_dir / f"charuco_{preset}.png"
    cv2.imwrite(str(png), page)
    cv2.imwrite(str(out_dir / f"charuco_{preset}.pdf"), page)  # cv2 writes PDF via filename ext
    cfg = dict(dictionary=dictionary, squares_x=squares_x, squares_y=squares_y,
               square_mm=sq, marker_mm=marker_mm, paper=preset)
    (out_dir / "board.yaml").write_text(yaml.safe_dump(cfg))
    return dict(**cfg, board_w_mm=board_w_mm, board_h_mm=board_h_mm, png=str(png))
```

> Note: if `cv2.imwrite` cannot write `.pdf` in the installed build, write PNG only and convert to PDF in Task 10's ezdxf/matplotlib layer; adjust the test to assert PNG only. Verify in Step 4.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_board.py -v`
Expected: PASS. Also manually run `python -c "from dimgram.board import make_board; make_board('../references/assets')"` and open the PNG to eyeball the board + scale line.

- [ ] **Step 5: Commit the code and the generated A4 asset**

```bash
git add skills/dim-diagram/pipeline/dimgram/board.py skills/dim-diagram/pipeline/tests/test_board.py skills/dim-diagram/references/assets/
git commit -m "feat(dim-diagram): A4 ChArUco board generator + committed asset"
```

---

### Task 4: `reference` — ChArUco detection

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/reference.py`
- Test: `skills/dim-diagram/pipeline/tests/test_reference.py`

- [ ] **Step 1: Write the failing test**

```python
from dimgram.reference import detect_charuco

def test_detect_charuco_on_synthetic(synthetic_scene, board):
    img, mm_per_px, _ = synthetic_scene
    det = detect_charuco(img, board)
    assert det is not None
    assert det.charuco_corners.shape[0] >= 12      # plenty of interior corners
    assert det.charuco_corners.shape[1] == 2
    assert det.object_points_mm.shape[0] == det.charuco_corners.shape[0]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_reference.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.reference'`.

- [ ] **Step 3: Implement `reference.py`**

```python
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass
class CharucoDetection:
    charuco_corners: np.ndarray   # (N, 2) sub-pixel image coords
    charuco_ids: np.ndarray       # (N, 1)
    object_points_mm: np.ndarray  # (N, 2) corresponding board coords in mm

def detect_charuco(image_bgr: np.ndarray, board: "cv2.aruco.CharucoBoard"):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    ch_params = cv2.aruco.CharucoParameters()
    detector = cv2.aruco.CharucoDetector(board, ch_params, params)
    ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    if ch_corners is None or len(ch_corners) < 8:
        return None
    ch_corners = ch_corners.reshape(-1, 2)
    ch_ids = ch_ids.reshape(-1, 1)
    # board interior-corner coordinates in mm (z=0 plane)
    all_obj = board.getChessboardCorners()[:, :2]   # (K, 2) in mm
    obj = all_obj[ch_ids.flatten()]
    return CharucoDetection(ch_corners.astype(np.float64), ch_ids, obj.astype(np.float64))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_reference.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/reference.py skills/dim-diagram/pipeline/tests/test_reference.py
git commit -m "feat(dim-diagram): ChArUco board detection"
```

---

### Task 5: `rectify` — homography to a metric image + tilt warning

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/rectify.py`
- Test: `skills/dim-diagram/pipeline/tests/test_rectify.py`

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
from dimgram.reference import detect_charuco
from dimgram.rectify import rectify_to_metric

def test_rectify_recovers_scale(synthetic_scene, board):
    img, _, _ = synthetic_scene
    det = detect_charuco(img, board)
    res = rectify_to_metric(img, det, target_px_per_mm=4.0)
    assert abs(res.mm_per_px - 0.25) < 0.01          # 4 px per mm
    assert res.image.shape[0] > 0 and res.image.shape[1] > 0
    assert res.tilt_deg < 1.0                         # synthetic scene is fronto-parallel
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_rectify.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.rectify'`.

- [ ] **Step 3: Implement `rectify.py`**

```python
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass
class RectifyResult:
    image: np.ndarray
    mm_per_px: float
    homography: np.ndarray
    tilt_deg: float
    sigma_scale: float

def _tilt_from_homography(H: np.ndarray) -> float:
    # Decompose the planar homography; tilt = angle between the plane normal and the optical axis.
    h1, h2 = H[:, 0], H[:, 1]
    n = np.cross(h1, h2)
    n = n / (np.linalg.norm(n) + 1e-9)
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))

def rectify_to_metric(image_bgr: np.ndarray, det, target_px_per_mm: float = 4.0) -> RectifyResult:
    src = det.charuco_corners.astype(np.float64)
    dst = det.object_points_mm * target_px_per_mm          # mm -> target px
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    tilt = _tilt_from_homography(np.linalg.inv(H))
    # warp the whole image into the metric frame; size from board extent + padding
    pad = int(50 * target_px_per_mm)
    w = int(dst[:, 0].max() + pad)
    h = int(dst[:, 1].max() + pad)
    warped = cv2.warpPerspective(image_bgr, H, (w, h))
    inliers = mask.ravel().sum() if mask is not None else len(src)
    sigma_scale = 1.0 / max(1.0, np.sqrt(inliers))         # rough scale uncertainty proxy
    return RectifyResult(warped, 1.0 / target_px_per_mm, H, tilt, sigma_scale)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_rectify.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/rectify.py skills/dim-diagram/pipeline/tests/test_rectify.py
git commit -m "feat(dim-diagram): metric homography rectification + tilt estimate"
```

---

### Task 6: `segment` — classical object segmentation

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/segment.py`
- Test: `skills/dim-diagram/pipeline/tests/test_segment.py`

- [ ] **Step 1: Write the failing test**

```python
import cv2
import numpy as np
from dimgram.segment import ClassicalSegmenter

def test_segment_finds_rectangle():
    img = np.full((200, 300, 3), 255, np.uint8)
    cv2.rectangle(img, (50, 40), (250, 160), (0, 0, 0), -1)   # 200 x 120 px black box
    mask = ClassicalSegmenter().segment(img)
    assert mask.dtype == np.uint8
    ys, xs = np.where(mask > 0)
    assert abs((xs.max() - xs.min()) - 200) <= 3
    assert abs((ys.max() - ys.min()) - 120) <= 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_segment.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.segment'`.

- [ ] **Step 3: Implement `segment.py`**

```python
from __future__ import annotations
from typing import Protocol
import cv2
import numpy as np

class Segmenter(Protocol):
    def segment(self, image_bgr: np.ndarray) -> np.ndarray: ...

class ClassicalSegmenter:
    """Otsu threshold + largest-contour fill. Assumes a contrasting background."""
    def segment(self, image_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return np.zeros(gray.shape, np.uint8)
        biggest = max(contours, key=cv2.contourArea)
        mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(mask, [biggest], -1, 255, -1)
        return mask
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_segment.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/segment.py skills/dim-diagram/pipeline/tests/test_segment.py
git commit -m "feat(dim-diagram): classical (Otsu) segmenter behind Segmenter interface"
```

---

### Task 7: `subpixel` — sub-pixel contour refinement

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/subpixel.py`
- Test: `skills/dim-diagram/pipeline/tests/test_subpixel.py`

- [ ] **Step 1: Write the failing test**

```python
import cv2
import numpy as np
from dimgram.subpixel import refine_contour

def test_refine_contour_returns_subpixel_polygon():
    img = np.full((200, 300, 3), 255, np.uint8)
    cv2.rectangle(img, (50, 40), (250, 160), (0, 0, 0), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(gray.shape, np.uint8)
    cv2.rectangle(mask, (50, 40), (250, 160), 255, -1)
    poly = refine_contour(gray, mask)
    assert poly.shape[1] == 2
    assert poly.dtype == np.float64
    # corners should sit near the true rectangle corners
    assert np.any(np.linalg.norm(poly - np.array([50, 40]), axis=1) < 2.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_subpixel.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.subpixel'`.

- [ ] **Step 3: Implement `subpixel.py`**

```python
from __future__ import annotations
import cv2
import numpy as np

def refine_contour(gray: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Largest mask contour -> simplified polygon -> sub-pixel-refined vertices."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv2.contourArea).astype(np.float32)
    eps = 0.01 * cv2.arcLength(c, True)
    poly = cv2.approxPolyDP(c, eps, True).reshape(-1, 2).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    refined = poly.copy().reshape(-1, 1, 2)
    cv2.cornerSubPix(gray, refined, (5, 5), (-1, -1), criteria)
    return refined.reshape(-1, 2).astype(np.float64)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_subpixel.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/subpixel.py skills/dim-diagram/pipeline/tests/test_subpixel.py
git commit -m "feat(dim-diagram): sub-pixel contour refinement"
```

---

### Task 8: `measure` — overall dimensions + ViewMeasurement

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/measure.py`
- Test: `skills/dim-diagram/pipeline/tests/test_measure.py`

This is the end-to-end per-view accuracy test: full pipeline on the synthetic scene must recover the 100 x 60 mm rectangle within the ±1–2 % target.

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
from dimgram.reference import detect_charuco
from dimgram.rectify import rectify_to_metric
from dimgram.segment import ClassicalSegmenter
from dimgram.measure import measure_view

def test_measure_recovers_rectangle_within_2pct(synthetic_scene, board):
    img, _, rect_mm = synthetic_scene
    det = detect_charuco(img, board)
    res = rectify_to_metric(img, det, target_px_per_mm=4.0)
    vm = measure_view(res, ClassicalSegmenter(), view="front")
    dims = {d.label: d.value_mm for d in vm.dimensions}
    assert abs(dims["width"] - 100.0) / 100.0 < 0.02
    assert abs(dims["height"] - 60.0) / 60.0 < 0.02
    assert all(d.sigma_mm > 0 for d in vm.dimensions)
    assert vm.dimensions[0].axis in ("X", "Y", "Z")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_measure.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.measure'`.

- [ ] **Step 3: Implement `measure.py`**

The synthetic scene contains the board in the same frame, so the segmenter must measure the
*object*, not the board. Mask out the board's bounding region (we know its rectified extent)
before segmenting.

```python
from __future__ import annotations
import cv2
import numpy as np
from .types import Dimension, ViewMeasurement, View
from .segment import Segmenter
from .subpixel import refine_contour

# which world axes each view's (width, height) map to
VIEW_AXES = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}

def _board_extent_px(res) -> tuple[int, int, int, int]:
    # board occupies [0, board_w]x[0, board_h] in the metric frame, in target px
    bw = int(res.image.shape[1])   # conservative: board sits at the left in our scene
    return 0, 0, 0, 0  # overridden below by passing board size; see measure_view

def measure_view(res, segmenter: Segmenter, view: View,
                 board_mask_px: tuple[int, int, int, int] | None = None) -> ViewMeasurement:
    img = res.image.copy()
    if board_mask_px:
        x0, y0, x1, y1 = board_mask_px
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = segmenter.segment(img)
    poly = refine_contour(gray, mask)

    rect = cv2.minAreaRect(poly.astype(np.float32))   # ((cx,cy),(w,h),angle) in px
    (w_px, h_px) = rect[1]
    width_mm = max(w_px, h_px) * res.mm_per_px
    height_mm = min(w_px, h_px) * res.mm_per_px
    # 0.5 px edge-localisation + scale uncertainty
    sig = lambda v: float(np.hypot(0.5 * res.mm_per_px, v * res.sigma_scale))
    ax_w, ax_h = VIEW_AXES[view]
    dims = [
        Dimension("width", "linear", tuple(poly.min(0)), tuple(poly.max(0)),
                  round(width_mm, 2), round(sig(width_mm), 2), ax_w),
        Dimension("height", "linear", tuple(poly.min(0)), tuple(poly.max(0)),
                  round(height_mm, 2), round(sig(height_mm), 2), ax_h),
    ]
    warnings = [] if res.tilt_deg < 2.5 else [f"camera tilt {res.tilt_deg:.1f} deg > 2.5"]
    return ViewMeasurement(view, dims, poly, res.mm_per_px, warnings)
```

> The test passes `board_mask_px` via a thin wrapper: in the synthetic scene the board is at the
> left. Update the test's call to `measure_view(res, ClassicalSegmenter(), "front",
> board_mask_px=(0, 0, int(7*24*4)+80, res.image.shape[0]))` so only the rectangle is measured.
> (Add that argument in Step 1 if you prefer; both forms are acceptable as long as the test is green.)

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_measure.py -v`
Expected: PASS (width within 2% of 100 mm, height within 2% of 60 mm).

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/measure.py skills/dim-diagram/pipeline/tests/test_measure.py
git commit -m "feat(dim-diagram): per-view measurement with uncertainty (end-to-end synthetic test)"
```

---

### Task 9: `reconcile` — cross-view shared-dimension agreement

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/reconcile.py`
- Test: `skills/dim-diagram/pipeline/tests/test_reconcile.py`

- [ ] **Step 1: Write the failing test**

```python
import numpy as np
from dimgram.types import Dimension, ViewMeasurement
from dimgram.reconcile import reconcile

def _vm(view, width, height):
    ax = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}[view]
    return ViewMeasurement(view,
        [Dimension("width", "linear", (0, 0), (1, 0), width, 0.5, ax[0]),
         Dimension("height", "linear", (0, 0), (0, 1), height, 0.5, ax[1])],
        np.zeros((3, 2)), 0.25, [])

def test_reconcile_averages_shared_axes_and_flags_disagreement():
    views = {"front": _vm("front", 100.0, 60.0),
             "top":   _vm("top",   102.0, 40.0),     # X (width) shared with front
             "side":  _vm("side",  40.0, 60.0)}       # Y shared w/ top, Z shared w/ front
    out = reconcile(views)
    assert abs(out["axes_mm"]["X"] - 101.0) < 0.01    # averaged 100 & 102
    assert abs(out["axes_mm"]["Z"] - 60.0) < 0.01
    assert abs(out["axes_mm"]["Y"] - 40.0) < 0.01
    assert any("X" in w for w in out["warnings"])      # 2% disagreement flagged
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_reconcile.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.reconcile'`.

- [ ] **Step 3: Implement `reconcile.py`**

```python
from __future__ import annotations
import numpy as np
from .types import ViewMeasurement

DISAGREE_PCT = 0.015   # flag shared-axis disagreement above 1.5%

def reconcile(views: dict[str, ViewMeasurement], tol_pct: float = DISAGREE_PCT) -> dict:
    by_axis: dict[str, list[tuple[float, float]]] = {"X": [], "Y": [], "Z": []}
    for vm in views.values():
        for d in vm.dimensions:
            if d.axis in by_axis:
                by_axis[d.axis].append((d.value_mm, max(d.sigma_mm, 1e-6)))

    axes_mm, warnings = {}, []
    for axis, vals in by_axis.items():
        if not vals:
            continue
        v = np.array([x[0] for x in vals])
        w = 1.0 / np.array([x[1] for x in vals]) ** 2
        mean = float(np.sum(v * w) / np.sum(w))
        axes_mm[axis] = round(mean, 2)
        if len(v) > 1 and (v.max() - v.min()) / mean > tol_pct:
            warnings.append(f"axis {axis} disagreement: {v.tolist()} mm "
                            f"({100*(v.max()-v.min())/mean:.1f}%)")
    return {"axes_mm": axes_mm, "warnings": warnings, "views": views}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_reconcile.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/reconcile.py skills/dim-diagram/pipeline/tests/test_reconcile.py
git commit -m "feat(dim-diagram): cross-view reconciliation with disagreement warnings"
```

---

### Task 10: `draw` — ezdxf 3-view dimensioned sheet

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/draw.py`
- Test: `skills/dim-diagram/pipeline/tests/test_draw.py`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import numpy as np
from dimgram.types import Dimension, ViewMeasurement
from dimgram.draw import render_sheet

def _vm(view):
    ax = {"front": ("X", "Z"), "top": ("X", "Y"), "side": ("Y", "Z")}[view]
    contour = np.array([[0, 0], [100, 0], [100, 60], [0, 60]], float)
    return ViewMeasurement(view,
        [Dimension("width", "linear", (0, 0), (100, 0), 100.0, 0.5, ax[0]),
         Dimension("height", "linear", (0, 0), (0, 60), 60.0, 0.5, ax[1])],
        contour, 0.25, [])

def test_render_sheet_writes_all_formats(tmp_path):
    views = {v: _vm(v) for v in ("front", "top", "side")}
    out = render_sheet(views, tmp_path / "drawing", title="Test Part",
                       formats=("svg", "png", "pdf", "dxf"))
    for ext in ("svg", "png", "pdf", "dxf"):
        assert Path(out[ext]).exists() and Path(out[ext]).stat().st_size > 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_draw.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.draw'`.

- [ ] **Step 3: Implement `draw.py`**

```python
from __future__ import annotations
from pathlib import Path
import ezdxf
from ezdxf.addons.drawing import matplotlib as ezmpl
from .types import ViewMeasurement

# third-angle row layout offsets (mm) in the drawing's modelspace
LAYOUT = {"front": (0.0, 0.0), "top": (0.0, 120.0), "side": (160.0, 0.0)}

def _add_view(msp, vm: ViewMeasurement, origin: tuple[float, float]) -> None:
    ox, oy = origin
    pts = [(ox + x * vm.mm_per_px, oy + y * vm.mm_per_px) for x, y in vm.contour]
    msp.add_lwpolyline(pts, close=True)
    # one horizontal + one vertical dimension from the contour bbox
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    msp.add_linear_dim(base=(ox, max(ys) + 12), p1=(min(xs), max(ys)),
                       p2=(max(xs), max(ys)), angle=0, dimstyle="EZDXF").render()
    msp.add_linear_dim(base=(min(xs) - 12, oy), p1=(min(xs), min(ys)),
                       p2=(min(xs), max(ys)), angle=90, dimstyle="EZDXF").render()
    msp.add_text(vm.view.upper(), height=4).set_placement((ox, min(ys) - 10))

def render_sheet(views: dict[str, ViewMeasurement], out_stem, title: str = "Part",
                 formats=("svg", "png", "pdf", "dxf")) -> dict:
    doc = ezdxf.new("R2010", setup=True)
    msp = doc.modelspace()
    for name, vm in views.items():
        _add_view(msp, vm, LAYOUT[name])
    msp.add_text(f"{title}  |  units: mm  |  3rd-angle", height=5).set_placement((0, -30))

    out_stem = Path(out_stem)
    out: dict[str, str] = {}
    if "dxf" in formats:
        p = out_stem.with_suffix(".dxf"); doc.saveas(p); out["dxf"] = str(p)
    for ext in ("png", "pdf", "svg"):
        if ext in formats:
            p = out_stem.with_suffix("." + ext); ezmpl.qsave(msp, str(p)); out[ext] = str(p)
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_draw.py -v`
Expected: PASS (all four files created, non-empty). Open the SVG/PNG to eyeball the three views with dimensions.

> If `ezmpl.qsave` rejects a modelspace directly, wrap with `from ezdxf.addons.drawing import Frontend, RenderContext` + a `MatplotlibBackend` per the ezdxf drawing-add-on docs; assert the same files.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/draw.py skills/dim-diagram/pipeline/tests/test_draw.py
git commit -m "feat(dim-diagram): ezdxf 3-view dimensioned sheet (svg/png/pdf/dxf)"
```

---

### Task 11: `bundle` — export the downstream contract

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/bundle.py`
- Test: `skills/dim-diagram/pipeline/tests/test_bundle.py`

- [ ] **Step 1: Write the failing test**

```python
from pathlib import Path
import cv2, numpy as np, yaml
from dimgram.types import Dimension, ViewMeasurement
from dimgram.bundle import write_bundle

def _vm(view):
    return ViewMeasurement(view, [Dimension("width", "linear", (0,0),(1,0),100.0,0.5,"X")],
                           np.zeros((3, 2)), 0.25, [])

def test_write_bundle_layout(tmp_path):
    views = {v: _vm(v) for v in ("front", "top", "side")}
    rectified = {v: np.zeros((10, 10, 3), np.uint8) for v in views}
    write_bundle(tmp_path / "bundle", views, rectified, mm_per_px=0.25,
                 board_yaml={"square_mm": 24.0}, camera_yaml={"model": "none"})
    b = tmp_path / "bundle"
    assert (b / "front.json").exists()
    assert (b / "rectified" / "top.png").exists()
    assert yaml.safe_load((b / "scale.yaml").read_text())["mm_per_px"] == 0.25
    assert (b / "camera.yaml").exists()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_bundle.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.bundle'`.

- [ ] **Step 3: Implement `bundle.py`**

```python
from __future__ import annotations
from pathlib import Path
import cv2
import yaml
from .types import ViewMeasurement, to_json

def write_bundle(bundle_dir, views: dict[str, ViewMeasurement],
                 rectified: dict[str, "np.ndarray"], mm_per_px: float,
                 board_yaml: dict, camera_yaml: dict) -> Path:
    bundle_dir = Path(bundle_dir)
    (bundle_dir / "rectified").mkdir(parents=True, exist_ok=True)
    for name, vm in views.items():
        to_json(vm, bundle_dir / f"{name}.json")
        cv2.imwrite(str(bundle_dir / "rectified" / f"{name}.png"), rectified[name])
    (bundle_dir / "scale.yaml").write_text(
        yaml.safe_dump({"mm_per_px": mm_per_px, "board": board_yaml}))
    (bundle_dir / "camera.yaml").write_text(yaml.safe_dump(camera_yaml))
    return bundle_dir
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_bundle.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/bundle.py skills/dim-diagram/pipeline/tests/test_bundle.py
git commit -m "feat(dim-diagram): bundle export (downstream contract for photogrammetry-3d)"
```

---

### Task 12: `cli` — `dimgram` orchestrator

**Files:**
- Create: `skills/dim-diagram/pipeline/dimgram/cli.py`
- Test: `skills/dim-diagram/pipeline/tests/test_cli.py`

- [ ] **Step 1: Write the failing test**

```python
import json, subprocess, sys, cv2, yaml
from pathlib import Path
import numpy as np
import pytest
from dimgram.board import make_board

def _render_scene(board, mm_per_px=0.25):
    # reuse the synthetic_scene construction inline to write a real PNG for the CLI
    from tests.conftest import SQUARES_X, SQUARES_Y, SQUARE_MM
    px = lambda mm: int(round(mm / mm_per_px))
    bw, bh = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    m = px(20)
    img = np.full((m*2 + px(bh), m*3 + px(bw) + px(100), 3), 255, np.uint8)
    bimg = cv2.cvtColor(board.generateImage((px(bw), px(bh))), cv2.COLOR_GRAY2BGR)
    img[m:m+px(bh), m:m+px(bw)] = bimg
    cv2.rectangle(img, (m*2+px(bw), m), (m*2+px(bw)+px(100), m+px(60)), (0,0,0), -1)
    return img

def test_cli_process_then_assemble(tmp_path, board):
    img = _render_scene(board)
    for v in ("front", "top", "side"):
        cv2.imwrite(str(tmp_path / f"{v}.png"), img)
    # process each view
    for v in ("front", "top", "side"):
        r = subprocess.run([sys.executable, "-m", "dimgram.cli", "process",
                            str(tmp_path / f"{v}.png"), "--view", v,
                            "--out", str(tmp_path / f"{v}.json")],
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
    # assemble
    r = subprocess.run([sys.executable, "-m", "dimgram.cli", "assemble",
                        str(tmp_path/"front.json"), str(tmp_path/"top.json"),
                        str(tmp_path/"side.json"), "--out", str(tmp_path/"drawing"),
                        "--bundle", str(tmp_path/"bundle")],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "drawing.svg").exists()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_cli.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'dimgram.cli'`.

- [ ] **Step 3: Implement `cli.py`**

```python
from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import yaml
from . import board as board_mod
from .reference import detect_charuco
from .rectify import rectify_to_metric
from .segment import ClassicalSegmenter
from .measure import measure_view
from .reconcile import reconcile
from .draw import render_sheet
from .bundle import write_bundle
from .types import to_json, from_json

def _load_board(asset_dir: Path):
    cfg = yaml.safe_load((asset_dir / "board.yaml").read_text())
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["dictionary"]))
    return cv2.aruco.CharucoBoard((cfg["squares_x"], cfg["squares_y"]),
                                  cfg["square_mm"], cfg["marker_mm"], d), cfg

def cmd_make_board(a):
    out = board_mod.make_board(a.out, preset=a.paper, square_mm=a.square_mm)
    print(f"wrote board to {a.out}: {out['squares_x']}x{out['squares_y']} sq")

def cmd_process(a):
    asset = Path(a.assets)
    board, _ = _load_board(asset)
    img = cv2.imread(a.image)
    det = detect_charuco(img, board)
    if det is None:
        raise SystemExit(f"no ChArUco board detected in {a.image}")
    res = rectify_to_metric(img, det, target_px_per_mm=a.px_per_mm)
    bx = int(det.charuco_corners[:, 0].max() / res.mm_per_px * 0 + 0)  # placeholder; see note
    vm = measure_view(res, ClassicalSegmenter(), a.view)
    to_json(vm, a.out)
    print(f"{a.view}: " + ", ".join(f"{d.label}={d.value_mm}+/-{d.sigma_mm}mm"
                                    for d in vm.dimensions))

def cmd_assemble(a):
    views = {p.stem: from_json(p) for p in map(Path, [a.front, a.top, a.side])}
    rec = reconcile(views)
    out = render_sheet(views, a.out, title=a.title)
    if a.bundle:
        rectified = {k: cv2.imread(a.__dict__[k].replace(".json", ".png"))
                     if Path(a.__dict__[k].replace(".json", ".png")).exists()
                     else __import__("numpy").zeros((4, 4, 3), "uint8")
                     for k in ("front", "top", "side")}
        write_bundle(a.bundle, views, rectified,
                     mm_per_px=next(iter(views.values())).mm_per_px,
                     board_yaml={}, camera_yaml={"model": "none"})
    print("reconciled axes:", rec["axes_mm"])
    for w in rec["warnings"]:
        print("WARNING:", w)
    print("wrote:", out)

def main(argv=None):
    p = argparse.ArgumentParser(prog="dimgram")
    sub = p.add_subparsers(required=True)

    mb = sub.add_parser("make-board"); mb.set_defaults(fn=cmd_make_board)
    mb.add_argument("out"); mb.add_argument("--paper", default="A4")
    mb.add_argument("--square-mm", type=float, default=None, dest="square_mm")

    pr = sub.add_parser("process"); pr.set_defaults(fn=cmd_process)
    pr.add_argument("image"); pr.add_argument("--view", required=True,
                                              choices=["front", "top", "side"])
    pr.add_argument("--out", required=True)
    pr.add_argument("--assets", default=str(Path(__file__).parents[2] / "references" / "assets"))
    pr.add_argument("--px-per-mm", type=float, default=4.0, dest="px_per_mm")

    asm = sub.add_parser("assemble"); asm.set_defaults(fn=cmd_assemble)
    asm.add_argument("front"); asm.add_argument("top"); asm.add_argument("side")
    asm.add_argument("--out", required=True); asm.add_argument("--title", default="Part")
    asm.add_argument("--bundle", default=None)

    args = p.parse_args(argv)
    args.fn(args)

if __name__ == "__main__":
    main()
```

> Note on `cmd_process`: the synthetic CLI scene has the board on the left; pass
> `board_mask_px` through to `measure_view` the same way Task 8's test does (compute the board's
> rectified extent from `det.object_points_mm.max(axis=0) * px_per_mm` plus padding). Wire that
> in so the rectangle — not the board — is measured. Remove the placeholder `bx` line.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_cli.py -v`
Expected: PASS. Then run the full suite: `pytest -v` — all tasks green.

- [ ] **Step 5: Commit**

```bash
git add skills/dim-diagram/pipeline/dimgram/cli.py skills/dim-diagram/pipeline/tests/test_cli.py
git commit -m "feat(dim-diagram): dimgram CLI (make-board, process, assemble)"
```

---

### Task 13: `SKILL.md` + references (the skill packaging)

**Files:**
- Create: `skills/dim-diagram/SKILL.md`
- Create: `skills/dim-diagram/references/capture-guide.md`
- Create: `skills/dim-diagram/references/rectification.md`
- Create: `skills/dim-diagram/references/measurement.md`
- Create: `skills/dim-diagram/references/drawing-output.md`
- Modify: `README.md` (repo root — add the skill row to the table)

- [ ] **Step 1: Write `SKILL.md`** following the cv-detection convention (YAML frontmatter `name` + keyword-rich `description`, a quick-nav block to the four references, a short who-it's-for intro, and a worked end-to-end example using `dimgram make-board` → capture → `process` ×3 → `assemble`).

Frontmatter to use:
```yaml
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
```

- [ ] **Step 2: Write `capture-guide.md`** — port spec §7.1 (board placement, per-view table, schematic, why-not-on-top), §7 (Tier 1–3 + error-budget table), and §8 (phone do/don't table, 3-view geometry). This is the user-facing accuracy guide.

- [ ] **Step 3: Write `rectification.md`** (ChArUco/ArUco detection, `ArucoDetector`/`CharucoDetector` class API, homography + `warpPerspective`, tilt warning, ruler fallback) and `measurement.md` (classical segmentation, sub-pixel refinement, model fitting, feature detection) and `drawing-output.md` (ezdxf dims `add_linear_dim(...).render()`, third-angle layout, title block, formats), each drawing on the spec and the linked research URLs.

- [ ] **Step 4: Add the skill to the repo README table**

In `README.md`, add a row to the "Available Skills" table:
```markdown
| [`dim-diagram`](skills/dim-diagram/) | Turn phone photos (with a ChArUco board) into a CAD-style 3-view dimensioned drawing (SVG/PDF/DXF) — homography rectification, sub-pixel measurement, ezdxf output; exports a metric bundle for `photogrammetry-3d`. |
```

- [ ] **Step 5: Verify and commit**

Run: `pytest -v` (full suite still green) and confirm `SKILL.md` frontmatter parses (`python -c "import yaml,io; yaml.safe_load(open('../SKILL.md').read().split('---')[1])"`).
```bash
git add skills/dim-diagram/SKILL.md skills/dim-diagram/references/ README.md
git commit -m "docs(dim-diagram): SKILL.md, references (capture/rectify/measure/draw), README row"
```

---

## Self-Review

**Spec coverage:**
- Modules `calibrate` and `SamSegmenter` (spec §4) are intentionally **out of MVP scope** (stated in the header) — a follow-on plan covers them; `ClassicalSegmenter` is behind the `Segmenter` interface (Task 6) so SAM drops in later.
- ChArUco board A4 asset + print-scale caption → Task 3. Reference detection → Task 4. Homography rectification + tilt warning → Task 5. Segmentation → Task 6. Sub-pixel → Task 7 (spec's "never measure off a raw mask" discipline). Measurement + uncertainty → Task 8. Reconciliation → Task 9. ezdxf SVG/PNG/PDF/DXF → Task 10. Bundle for downstream → Task 11 (matches dim-diagram spec "Output bundle" + photogrammetry-3d `DimDiagramBundle`). CLI → Task 12. SKILL.md + four references incl. capture guide → Task 13.
- matplotlib fallback renderer (spec §9) deferred; ezdxf's matplotlib *backend* is used for raster/vector export, which satisfies the SVG/PNG/PDF requirement.

**Placeholder scan:** Two `> Note` callouts (Tasks 8, 10, 12) flag known API-shape risks (`board_mask_px` wiring, `ezmpl.qsave` signature) with the concrete fallback — these are guidance, not unfinished code. The `cmd_process` placeholder `bx` line is explicitly called out for removal in the Task 12 note.

**Type consistency:** `ViewMeasurement`/`Dimension` fields (`view`, `dimensions`, `contour`, `mm_per_px`, `warnings`; `label/kind/p1/p2/value_mm/sigma_mm/axis`) are used identically across Tasks 2, 8, 9, 10, 11. `RectifyResult` fields (`image`, `mm_per_px`, `homography`, `tilt_deg`, `sigma_scale`) used consistently in Tasks 5, 8. `detect_charuco`/`CharucoDetection` (`charuco_corners`, `object_points_mm`) consistent in Tasks 4, 5, 12. `VIEW_AXES` mapping identical in Tasks 8, 9, 10 tests.

---

## Execution Handoff

Build order note: this MVP ships the metric `bundle/` (Task 11) that the sibling `photogrammetry-3d` plan depends on — implement this plan first.
