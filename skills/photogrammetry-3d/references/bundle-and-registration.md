# Consuming the dim-diagram bundle; registration & carving

This is what makes the combined pipeline metric and robust: the `dim-diagram` **bundle** is used
as priors and ground truth, not re-derived.

## The bundle (input contract)

`dim-diagram`'s `assemble --bundle ./bundle` writes:

```
bundle/
  front.json  top.json  side.json     # ViewMeasurement: dims + sub-pixel silhouette contour
  rectified/{front,top,side}.png       # metric rectified views
  scale.yaml                           # mm_per_px + board geometry
  camera.yaml                          # intrinsics + distortion (or "model: none")
```

In Python, read these into a `DimDiagramBundle` (views, rectified paths, `mm_per_px`,
`board_pose`, `camera_yaml`). The schema mirrors `dim-diagram`'s `ViewMeasurement`.

## Registration (scale + canonical alignment)

COLMAP output is up-to-scale and arbitrarily oriented. Register it onto the bundle's metric,
canonical frame:

1. Detect the **shared ChArUco markers** in the photo set (`cv2.aruco`, same board as the
   bundle).
2. Triangulate the marker corners into the reconstruction using the COLMAP poses.
3. Solve the **similarity transform** (scale + rotation + translation) mapping reconstruction
   coordinates to the board's metric coordinates — i.e. the same coordinate frame `dim-diagram`
   used. This applies **both the absolute scale and the front/top/side axes** at once, replacing
   any self-derived scale or PCA-based alignment.

The board's known square size (verified with calipers upstream) means scale is trustworthy and
consistent between the 2D drawing and the 3D mesh.

## Silhouette carving (visual hull)

The bundle carries three **metric silhouettes** (the rectified-view object contours). Use them
as a **visual-hull bound**: back-project each silhouette into 3D and intersect to get a
conservative hull. Then:

- **Clip** mesh geometry that falls outside the hull (stray points, background remnants).
- **Flag / fill** texture-poor regions where MVS produced holes but the hull says solid.

This is optional but high-value when surfaces reconstruct poorly.

## References
- ArUco detection: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- Visual hull / shape-from-silhouette: https://en.wikipedia.org/wiki/Visual_hull
