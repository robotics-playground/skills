# Rectification: ChArUco detection, homography, scale

This stage turns a perspective photo into a **fronto-parallel, metric** image where a fixed
number of pixels equals one millimetre. It is implemented in `dimgram/reference.py` and
`dimgram/rectify.py`.

## Why a ChArUco board (not a bare ruler)

- A **ruler** gives only a 1-D scale factor and assumes the camera is perfectly square-on. No
  perspective correction.
- A **ChArUco board** (chessboard + ArUco markers) gives many corners with **known board
  coordinates in mm**, which is exactly what a planar **homography** needs. The chessboard
  saddle-corners localise to true sub-pixel precision, and the ArUco markers identify the board
  and tolerate partial occlusion. It also lets you **calibrate the camera for free**.

## Detection (OpenCV ≥ 4.7 class API)

`estimatePoseSingleMarkers` is **deprecated** — use the class-based detector:

```python
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG   # sharpest corners
detector = cv2.aruco.CharucoDetector(board, cv2.aruco.CharucoParameters(), params)
charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
```

`board.getChessboardCorners()` returns each interior corner's (x, y, z) position in mm
(z = 0). Indexing it by `charuco_ids` gives the **image-corner ↔ mm-corner** correspondence.

## Homography to a metric image

```python
dst = object_points_mm * target_px_per_mm          # mm -> target px (e.g. 4 px/mm)
H, mask = cv2.findHomography(src_corners, dst, cv2.RANSAC, 3.0)
warped = cv2.warpPerspective(image, T @ H, (out_w, out_h))
```

Implementation notes from `rectify.py`:
- The output canvas is sized to the **full transformed input image** (with a translation `T` so
  coordinates stay positive) — otherwise the object, which sits *outside* the board's extent, is
  clipped.
- `mm_per_px = 1 / target_px_per_mm`. All later measurements multiply pixel distances by this.
- **Scale uncertainty** `sigma_scale` is the homography reprojection RMS divided by the board
  span — a small fraction for a clean detection, larger when the fit is poor.

## Camera squareness / tilt warning

A tilted sensor causes keystone error (scale varies across the frame). `rectify.py` estimates a
**tilt proxy** from how far the metric→image map departs from a similarity (fronto-parallel →
~0°). `measure.py` attaches a warning when tilt ≥ 2.5°. Mitigate with a tripod + level; recover
squareness by re-shooting, not in post.

## Camera calibration (implemented)

Undistort before measuring to remove ~1 % edge error. Implemented in `dimgram/calibrate.py`
and wired into the CLI:
1. Shoot 10–20 views of the same ChArUco board at varied angles/distances into a folder.
2. `dimgram calibrate <folder> --out camera.yaml` — runs `CharucoDetector.detectBoard` +
   `board.matchImagePoints` + `cv2.calibrateCamera`, writes `camera.yaml`, and reports the
   RMS reprojection error (target **< 0.3 px**, acceptable < 1 px).
3. `dimgram process photo.jpg --view front --camera camera.yaml ...` undistorts
   (`cv2.undistort`) **before** detection.

> Calibration is optional: with no `--camera`, the pipeline assumes no distortion — keep the
> object near the frame centre where distortion is smallest.

## Ruler fallback

If only a ruler is available: detect its known length to set `mm_per_px` (scale only, no
perspective correction). Expect several-percent error off-axis. Strongly prefer the board.

## References
- ArUco detection: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- ChArUco calibration: https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html
