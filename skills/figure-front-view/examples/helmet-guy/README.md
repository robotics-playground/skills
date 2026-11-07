# Worked example: "Helmet Guy" (Vinc Studio squat figure)

Record of the scripts that produced the final front view (12 parts, outline IoU 0.98 vs artwork).
They are **one-off project scripts**: crop window, palette, row/column windows and file paths are
hard-coded for this toy, and they read the artwork/photos from the session upload folder (not
included here - third-party character). Reuse the structure and the decisions, not the numbers;
generic, tested building blocks live in `../../pipeline/figfront/`.

## Run order
| Step | Script | Does |
|---|---|---|
| 1 | `model.py` | Part primitives (shapely) + stacking order (`ORDER`, delivered `OUT_ORDER` with SUIT = jacket + pants) |
| 2 | `fit.py` | Colour unmixing of the artwork crop; staged Nelder-Mead fit of every primitive (head / body / legs axes) -> `params/fit.json` |
| 3 | `measure_px.py` | Pixel calibration: edge spread vs simulated ideal edge, edge noise -> `params/pixel_metrics.json` |
| 4 | `trace.py` | Semantic targets, partition of black parts by primitives, leftovers, exact arms ring / straight jacket / straight-V cravat, hidden completion -> `full_masks.npz` |
| 5 | `photo_face.py`, `fit_face.py` | Face measurements on the print photo (relative to the helmet) - used while exploring the face; final face uses the artwork chin circle |
| 6 | `smooth.py` | Mask -> bounded-smoothed outlines with real-corner rebuild; exact geometry overrides (arms, jacket, cravat), suit merge, waist corners, leg gap + leg edges, face = hidden face shape + measured chin circle -> `smooth.pkl` |
| 7 | `export2.py` | mm transform, `parts_mm.json`, DXF (splines, per part + all layers + visible regions), SVG front view + parts sheet |
| 8 | `build_page.py` + `page_template.html` | Review page: overlay slider over the artwork pixels, dimensioned view, parts cards |

## Key numbers (params/)
- `pixel_metrics.json`: 0.3152 mm/px nominal, effective pixel 1.33 px (0.42 mm), noise 0.026 px,
  max shift 0.333 px (0.105 mm).
- `cravat_poly.json`: straight-line V (width 3.08 -> 4.78 mm, V 4.69 mm deep, 54 deg).
- `chin_edge.json`: chin circle R 54.7 px (17.2 mm), fit rms 0.07 px.
- `fit.json`: all primitive parameters (arm ring outer 30.0 x 17.7 mm half-axes, inner 18.2 x 11.5).

## What to change for the next toy
See `../../references/lessons-learned.md` -> "Try next time": move palette points, crop, scale,
parts, stacking order and a per-edge spec into one config file, and replace the per-part special
cases in `smooth.py` with edge-spec driven geometry.
