# Worked example: "Fan figure" (second project, same squat figure)

Config-driven, staged scripts that produced the final front view in 4 user rounds after round 0.
Start here for a new toy: copy the structure, then change `config.py` (inputs, palette, stacking order,
scale) and the feature windows in the stages. Coordinates and row/column windows are for this one
artwork; generic, tested code lives in `../../pipeline/figfront/` (`outline`, `strokes`, `pixels`,
`checks` were written for this project).

| Step | Script | Does |
|---|---|---|
| 1 | `stage1.py` | Unmix colours on the 6x grid; base / sole line; mirror axis per zone (head, torso, legs) |
| 2 | `stage2.py` | Symmetric masks (legs moved onto the body axis); face and shirt split at the neck; face details aligned + RAW copies; arm-ring ellipses from the ring holes and silhouette; jacket width |
| 3 | `stage3.py` | Calibration; helmet, face (chin arc + hidden head), cravat V, hands (two fist arcs), arms (ellipses + tangent ends on the hand sides), body (exact jacket + traced legs + measured boot staircase, spliced onto the previous round), face details as drawn (strokes), base |
| 4 | `stage4.py` | mm transform; checks; `parts_mm.json`; DXF (LINE / ARC / ELLIPSE) + visible-region DXF; SVG views; numbered edge sheet |
| - | `render.py` | Overlays on the artwork (legs moved onto the axis), assembled view, zooms |
| - | `build_page.py` | Review page (Artifact): changes block, overlay slider, edge table, parts, checks, questions |

Round-to-round scope guard: copy `work/parts_px.pkl` to `work/parts_px_roundN.pkl` before a change;
`stage3.py` splices the BODY onto `parts_px_round1.pkl` above the boot zone (method 9a). `part_spec.md`
is the final spec. The artwork and print photos are not included (Vinc Studio character).
