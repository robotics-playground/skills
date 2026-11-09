# Method (technical)

All coordinates below are image pixels of the straight-on artwork (y down) unless stated; the
working grid is the crop upsampled `UP = 6x` (cubic) so masks have sub-pixel resolution.

## 1. Understanding and part spec
Write `part_spec.md` (template: `part-spec-template.md`). For each part: colour, primitive, parameters,
relations, and whether its outline is **traced**, **exact geometry** or **photo-informed hidden**.
Read the stacking order from seams on the print photo (a separate piece shows a groove/edge where it
lies on another). Typical order for a front-facing figure, back -> front:
`shirt -> suit (jacket+pants) -> boots -> arm ring -> hands -> face -> helmet/hair -> face lines`.

## 2. Scale
`mm_per_px = known_mm / measured_px` from one real dimension (figure height, base width). Weakest link:
ask for a caliper number. Report every dimension in mm with this assumption stated.

## 3. Colour unmixing (`figfront.unmix.classify`)
Palette = flat paints sampled from the artwork (background included; add light-grey if hands differ
from face white). Blur 0.3 original px (0.45 merged a 1 px mouth gap). Each pixel: nearest flat paint,
or the best two-paint blend `p_i + a (p_j - p_i)`; label = majority paint. Take the largest connected
component per paint for big parts (mixed pixels elsewhere can match a paint).

## 4. Pixel calibration (`figfront.calibrate`)
1. Edge widths: sample colour along normals of a clean edge (helmet|background, black|background),
   10-90 % of the A->B blend.
2. Simulate an ideal anti-aliased step at source pixel sizes 1..3 px, resampled to the grid and
   measured the same way (bilinear sampling alone gives ~1.4 px for a perfect edge).
3. Effective pixel = interpolate measured width in the simulated table (1.6 px measured -> 1.33 px).
4. Noise = RMS of the sub-pixel contour around local quadratics (+-1.5 effective px) -> ~0.03 px
   after unmixing.
5. Budget: max shift = 1/4 effective px; sigma = max(noise, 1/8 effective px); corner scale = 1 effective px.

## 5. Structure fit (primitives)
Fit simple primitives per part by IoU on its colour mask (Nelder-Mead with explicit initial simplex;
default simplex steps are too small for translations). Group axes (head / body / legs) can differ by
~1 mm in hand-drawn art; record the offset, decide symmetry with the user.
Primitives used: ellipse-cut helmet with filleted flaps, squircle face, superellipse lenses, arc
strokes, pentagon V, rounded-rect hands + thumbs, oval ring arms (outer - inner), rectangle jacket,
pelvis + tapered legs + gap, boots = ankle block + rounded toe.

## 6. Partition, leftovers, hidden completion (mask level)
- Same-colour regions (all black parts) -> label by primitive, front-most first; pixels outside every
  primitive -> nearest primitive (distance transform).
- Leftover anti-aliased pixels -> nearest visible part, but **never within 1 px of background** and,
  inside the head zone, only to head parts.
- Hidden completion: `full = vis | ((prim | grow(vis, 3px)) & grow(front, 0.5px))`, then seam-weld
  (close/open) only near front parts (`figfront.layers`).
- Face graphics (glasses, brows, nose, mouth): dark components inside the face hull, each assigned to
  the detail primitive it overlaps most; keep every stroke (e.g. the lower-lip line).

## 7. Outlines (`figfront.contours`)
Sub-pixel contours (Gaussian 0.23 px + marching squares) -> resample 0.25 px -> real corners
(turning >= 32 deg over 1 effective px, both sides straight (rms < 1/8 eff px), sides keep their
direction, sides meet at >= 32 deg) -> rebuild corners at side-line intersections -> per-segment
spline with sigma, accepted only if Hausdorff <= max shift (else less smoothing).

## 7a. Specks
After tracing and before regularizing, run `contours.drop_specks(part, min_area)` on every part with
`min_area = max(0.3 mm^2, (2 effective px)^2)` in the working units. Detail-rich parts (skin, hair,
logos) otherwise keep dozens of compression islands and pinholes. A part that is itself tiny (nose dot,
eye glint) is kept as one piece. Verify with `checks.specks(parts, min_area) == {}`.

## 7b. Regularization (`figfront.regularize`)
Per boundary run between real corners, accept the simplest candidate that stays within the regularity
tolerance (1/2 effective px) AND leaves no coherent one-signed residual longer than ~6x tol with a peak
above 1/4 effective px (that would be a flattened drawn feature):
`line < arc < line/arc pieces (dynamic programming, tangent joins) < cubic spline with fewest knots`.
Output: regularized polygon + an edge report (kind, parameters, max deviation) used for the numbered
review sheet (`figfront.review.review_sheet`). Mirror-symmetric edge pairs (`regularize.mirror_pairs`)
are proposed to the user, not applied silently.

## 7c. Outline builder: lines + tangent arcs (`figfront.outline`)
Users who rebuild in CAD want LINE and ARC entities, sharp only at real corners. `outline.build(ring, m)`:
corners as in 7 (plus forced vertices `corners_at` for exact geometry such as a V-neck), then per run:
one line or one arc if faithful (regularity tolerance), else a biarc chain (arcs meeting tangent) fitted to
the lightly smoothed trace, split at the worst point until within tolerance; near-flat arcs become lines.
Never use regularize's line/arc "pieces": their joins are not tangent (visible kinks on hips and legs).
Check with `outline.tangent_breaks(segs)`: smooth joins < 1 deg, corners > 20 deg.

## 7d. Symmetry (decide per part, by measurement)
- Symmetric parts: average the mask with its mirror (legs drawn off-axis are first moved onto the body
  axis), then `outline.symmetric_part`: the right half is fitted and mirrored; ends that belong on the axis
  are snapped onto it and the halves welded. Bugs seen: the open half-chain must keep its last point, and
  runs must wrap forward once around the ring, else thin wedges/slits appear along the axis.
- Details: `checks.mirror_offset(mask, axis, scale)`. Offset > ~0.5 effective px = drawn asymmetric (a face
  drawn slightly turned): keep those details as drawn, from the RAW masks. Mirror averaging of offset thin
  strokes shrinks and moves them. Default for faces: helmet/face outline symmetric, details as drawn; confirm
  in round 1.

## 7e. Small drawn features at the resolution limit
The corner test needs straight sides of >= 1 effective px; tracing blurs 1-2 px; smoothing (sigma ~1 eff px)
flattens the rest. So:
- **Short steps** (ankle ledges 2-4 px, 1-3 px steps): measure them on the pixels with
  `pixels.coverage` + `row_crossing` / `col_crossing` (50 % crossings, ~0.1 px), on both mirrored copies,
  averaged; build the staircase from lines with sharp corners. For a gentler fix, `outline.build(...,
  fine_zone=(y0, y1))` adds Douglas-Peucker step corners (turn >= 40 deg, sides >= 1.2 px) and fits that zone
  at the fidelity budget without smoothing. Do not run disk openings over such a zone.
- **Touching shapes** (two fists, two feet): measure the edge profile per column; one arc per shape through
  the shared notch point and the outer corner, best circle in between; sharp notch on the axis.
- **Thin lines** (brows, lips): `strokes.line_stroke` or `strokes.arc_stroke` (centre line from per-column
  centroids, width from area, round ends). Ask "straight or curved?" if the centre-line radius is large.

## 7f. Joins between parts that move together
Arm ends finish exactly on the hand side edge (both corners), so arms + hands are one closed loop. Keep the
visible ring edges as the free-fit ellipses; inside the zone where the artwork is black on black (ring over
the jacket) leave each ellipse tangentially and run a biarc into the hand corner, meeting it at the
ellipse's own angle. Forcing the whole ellipse through the corners moved the visible edge 0.5 mm.

## 8. Exact geometry (only where the print shows it)
- **Oval ring** (`geom.oval_ring`): fit outer/inner ellipse centres + radii to the silhouette.
- **Straight V** (`geom.straight_v`): fit centre, width at chin line, width at V start, V start, tip to
  the visible white pixels; reuse the same polygon as the jacket's V-neck cut (shared edge, no gap).
- **Jacket**: rectangle from the fit minus the V.
- **Leg gap**: inner edges `x(y)` quadratics + top edge `flat_with_bulge`; corners = intersections.
- **Outer leg edges**: `geom.spline_edge` (4 interior knots) from the waist corner to just above the
  ankle corner (pinned ends keep both corners sharp).
- **Waist corner**: extend the hip tangent (3.5-8 px past the fillet) to the jacket side line
  (`geom.corner_from_tangent`); snap the jacket line straight above it.
- **Chin**: measure the face|jacket edge column by column between the flap tips, excluding the shirt
  columns; `geom.fit_circle` (rms 0.07 px here). Join to the unchanged upper face at the y where both
  outlines have equal width (`geom.join_circles_y`).
- **Hidden face**: `geom.face_shape` sized from the eye line (glasses), visible width + tuck, and the
  helmet opening; only the part covered by the helmet is added.

## 9. Verify every round (self-review before the user sees anything)
Numbers: `checks.specks` (islands/pinholes), `checks.changed_parts` (scope), `checks.overlap_report` (back parts reach >= 1.5-2 mm under
front parts), `checks.gaps_and_slivers`, `checks.thin_features`, `checks.same_colour_neighbours`.
Pictures: answer every item of `checks.SELF_REVIEW_QUESTIONS` on zoomed pixel overlays.
- Pixel overlays per changed feature (nearest-neighbour zoom x5..x14, outline in red).
- Numbers: IoU per part vs its colour mask, whole-silhouette IoU, max shift (mm) per part,
  slivers < 0.3 mm^2 in visible regions, features thinner than 0.8 mm (nozzle 0.4 mm).
- Untouched-part diff (see lessons-learned, Try #6).

## 9a. Scope-safe edits (one zone of one part)
Re-running the builder on a part refits whole runs: fixing a boot moved the hip 0.3 mm. Instead keep the
approved segment chain, split it where it enters the asked zone (`y` band), and bridge with one tangent arc
(`outline.arc_from`) into the new zone segments; corners stay where the zone has them. Prove it with
`checks.changed_outside(prev, new, (y0, y1)) == 0` plus `changed_parts` for the other parts. Keep the
previous version's pickle (`parts_px_roundN.pkl`) for this.

## 10. Export
- mm transform: centre the figure axis on the base, sole line = base top.
- `*_parts.dxf`: complete parts per layer from the outline segments: LINE, ARC (y flips: angle -> -angle,
  sweep sign flips) and ELLIPSE with start/end parameters for exact ovals. Per-part DXFs too.
- `*_print_regions_visible.dxf`: `layers.visible_regions` in stacking order.
- SVG: assembled view with dimensions (overall, plate, base, head width, arm span) and a parts sheet.
- Review page: overlay slider over the artwork pixels, highlight-on-hover from the parts list.
