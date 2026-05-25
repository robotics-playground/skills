# Measurement: segmentation, sub-pixel edges, model fitting

Turns a rectified metric image into dimensions. Implemented in `dimgram/segment.py`,
`dimgram/subpixel.py`, `dimgram/measure.py`.

## The golden rule: segment to *localise*, refine to *measure*

**Never read a dimension off a raw mask.** SAM masks, GrabCut masks, Hough circles and
`minAreaRect` are all **pixel-quantised** — fine for *locating* a feature, wrong for *measuring*
it. Pixel quantisation is the dominant software error. The correct flow:

```
segment (localise) ─► refine contour to sub-pixel ─► fit a geometric model ─► dimension
```

## Segmentation (swappable `Segmenter` interface)

`segment.py` defines a `Segmenter` Protocol; the measurement core never changes when you swap
backends.

- **`ClassicalSegmenter` (default, shipped):** Gaussian blur → Otsu inverse threshold →
  largest external contour fill. Zero heavy deps. **Assumes a contrasting background** — hence
  the capture guide's matte-backdrop advice.
- **`SamSegmenter` (optional, planned, `[sam]` extra):** MobileSAM via `ultralytics` (~40 MB,
  CPU-viable) with a box/point prompt for cluttered backgrounds. Still only *localises* — the
  measurement is done by the sub-pixel stage below.

Classical options for harder cases (documented for when you extend it): `cv2.grabCut`
(cluttered, seed with a box), `cv2.adaptiveThreshold` (uneven light), `cv2.watershed` (touching
parts).

## Sub-pixel refinement (the metrology stage)

`subpixel.refine_contour(gray, mask)`:
1. `cv2.findContours` on the mask → largest contour.
2. `cv2.approxPolyDP` → clean polygon vertices.
3. `cv2.cornerSubPix` → refine vertices to sub-pixel using local gradients.

For curved edges or higher precision, refine along the edge normal (Steger/Devernay
Hessian-Taylor) and **fit a model** (`cv2.fitLine`, least-squares circle, `cv2.fitEllipse`) to
many sub-pixel points — fitting averages out noise far better than any single estimate.

## Overall dimensions

`measure.py`:
- Excludes the board region (its rectified bounding box, passed in) so only the object is
  measured.
- `cv2.minAreaRect(subpixel_polygon)` → rotation-invariant `(w, h)` in px → `× mm_per_px`.
- `width = max(w, h)`, `height = min(w, h)`; mapped to world axes per view
  (`front`→X,Z · `top`→X,Y · `side`→Y,Z).
- **Per-dimension uncertainty** `sigma_mm = hypot(0.5 px · mm_per_px, value · sigma_scale)` —
  combining edge localisation with the homography scale uncertainty.

## Features (holes, slots) — extension points

For dimensioning beyond the bounding box, seed with detectors then fit:
- Holes: `cv2.HoughCircles` on the grayscale to *locate*, then fit a circle to sub-pixel edge
  points for the diameter.
- Slots / straight edges: `cv2.HoughLinesP` on a Canny edge image to *locate*, then `cv2.fitLine`
  on sub-pixel points and measure the normal distance.

## References
- Sub-pixel edge contour (Steger): https://github.com/raymondngiam/subpixel-edge-contour-in-opencv
- Object size measurement: https://pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
- MobileSAM: https://docs.ultralytics.com/models/mobile-sam
