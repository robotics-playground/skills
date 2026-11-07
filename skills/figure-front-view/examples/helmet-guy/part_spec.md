# Part specification — "Helmet guy" squat figure (Vinc Studio design)

Step 1 of the reverse-engineering: what the object *is*, before any pixel fitting.

## Object understanding
- **Type:** desk figure, FDM 3D print, multi-colour (blue / white / black PLA).
- **Construction:** a flat figure plate of constant thickness (2.5D extrusion), standing in a
  slot of a rectangular base block. Colour regions are separate bodies printed side by side
  (multi-material), so every colour boundary is a real body boundary.
- **Mechanism (from sheet panel 6, arrows):** the arm ring swings/rotates — modelled as its
  own part (ARMS) so a pivot can be added later.
- **Design rule:** mirror-symmetric about one vertical axis (character is front-facing,
  squatting). Face graphics (mouth) may be the only intentional asymmetry.
- **Authority:** artwork (sheet image) = shape & proportion intent (wins on conflict);
  measured print = overall scale (plate height 99.9 mm, base 70 × 20.3 mm).

## Parts and primitives
| # | Part | Colour | Primitive model | Parameters | Relationships |
|---|------|--------|-----------------|------------|---------------|
| 1 | HELMET | blue | ellipse cut by a horizontal line, flap ends filleted, minus face opening | cx, cy, rx, ry, cut_y, flap_r | sits on head; opening = FACE |
| 2 | FACE | white | **visible bottom border = one circular arc** (chin circle R 17.23 mm, measured on the artwork's face/jacket edge between the flap tips, shirt columns excluded; fit error 0.07 px); **hidden part under the helmet = normal face shape** (rounded forehead, temples; unchanged), joined to the chin circle at flap-tip level where both are the same width | chin circle centre + radius, forehead height | BEHIND the HELMET (helmet on top); in front of ARMS, SUIT, CRAVAT; face lines on top |
| 3 | GLASSES | black | 2 superellipse lenses (mirrored, tilted) + bridge bar | dx, cy, w, h, n, tilt, bridge_t | on FACE, symmetric |
| 4 | EYEBROWS | black | 2 thick circular-arc strokes (mirrored) | dx, cy, r, span, t | above GLASSES |
| 5 | NOSE | black | ellipse | cy, rx, ry | on axis |
| 6 | MOUTH | black | thick arc stroke | cx, cy, half_len, sag, t | below NOSE (may be off-axis) |
| 7 | CRAVAT (shirt / tie) | white | **straight-line V**: straight sides widening downward (3.08 mm at the chin to 4.78 mm, 8.2° each side), then two straight edges to a sharp point (4.69 mm deep, 54°) | xc, w_chin, w_v, y_v, y_tip | BEHIND JACKET, visible through its V-neck; **exact geometry** |
| 8 | HANDS | white | rounded rectangle + 2 thumb tabs (clasped hands) | w, top_y, bot_y, r, thumb_dx, thumb_w, thumb_top | on the FRONT of the ARMS' lower band, on axis |
| 9 | ARMS | black | **one oval ring: outer oval − inner oval**, centred on the axis | cy, rx_o, ry_o, cy_i, rx_i, ry_i | top BEHIND head; lower band IN FRONT of JACKET, the inner oval's lower arc is the seam (print photo); carries HANDS; **exact geometry**; rotating piece |
| 10 | SUIT (jacket + pants, one piece) | black | jacket: **straight-sided rectangle** minus the cravat V; pants: traced pelvis + 2 legs + crotch gap; merged with no seam, waist step filled with a sharp corner where the straight jacket side meets the hip (hip tangent extended to the jacket line), gap between the legs rebuilt from three fitted curves (two inner leg edges x(y), top edge y(x): flat with a broad round-bottomed bulge in the middle, 0.35 mm deep and 8.8 mm wide, as drawn) meeting at sharp corners; each outer edge (waist corner to ankle) one continuous fitted spline (print shows smooth edges; max shift 0.21 mm) | jacket w/top/bot, pants hip/ankle/crotch | over CRAVAT, behind ARMS, legs end at BOOTS |
| 11 | BOOTS (×2) | black | ankle block + rounded toe block (mirrored, toes point outward) | inner_x, ankle_w, top_y, toe_x, toe_top, toe_r | stand on BASE top |
| 12 | BASE | black | rectangle (+ slot for plate tab) | w, h | plate tab in slot (depth assumed) |

## Stacking order (back → front), from seams visible on the print photo
CRAVAT → SUIT (jacket + pants) → BOOTS → ARMS → HANDS → FACE → HELMET → GLASSES / EYEBROWS / NOSE / MOUTH

## Geometry rule (human-shaped toy: shape detail, pose and face are kept)
- Visible outline of every part = sub-pixel trace of the artwork (no simplification, no symmetrisation,
  no thickening). Pose, boot shape, hand/thumb shape, hip curve and every face line (incl. the lower-lip
  stroke) are kept exactly as drawn.
- The fitted primitives above are used only to (1) decide which part each artwork pixel belongs to and
  (2) complete edges that are hidden behind a part in front (e.g. arm ring behind the head, jacket behind
  the arm ring). They never replace a visible edge.
- Features thinner than 0.8 mm (eyebrows, mouth) are flagged for printing, not modified.

## Raster-defect removal (the only thing removed), calibrated from the image
Measured by `measure_px.py` (`pixel_metrics.json`):
- Pixel width: 0.3152 mm per artwork pixel (from figure height 99.9 mm).
- Edge spread (10–90 %): 1.6 px; a perfect anti-aliased edge sampled the same way gives 1.4 px,
  so the image's effective pixel is 1.33 px = 0.42 mm (slightly softened, not enlarged).
- Edge noise after colour unmixing: 0.026 px rms (0.008 mm).

Settings derived from these:
1. Colour unmixing: each edge pixel is a blend of two paints and goes to the paint it is mostly made of.
2. Compression specks on the figure edge are never attached to a part; parts tuck up to 3 px under the part in front (no slivers).
3. Smoothing sigma = max(noise, 1/8 effective px) = 0.166 px; hard cap on edge movement =
   1/4 effective px = 0.333 px (0.105 mm), the precision limit of a sub-pixel edge.
4. Corners are judged over one effective pixel. A point is a real corner only if both sides are straight
   (rms < 1/8 effective px) and keep their direction (an arc keeps turning, so round thumb tips stay round).
   Real corners are rebuilt sharp by intersecting the two straight sides (tip moves at most one effective pixel).
5. DXF output: true SPLINE curves split at the real corners.

## Exact geometry where the real object is geometric
The print photo shows the arms as one smooth oval ring and the jacket sides as straight lines. Small bumps on
those edges in the artwork are drawing jitter, so these two parts use the fitted geometry exactly
(arms: one analytic oval ring, outer oval minus inner oval, exported as true curves; jacket: rectangle with the cravat's V cut out; cravat: straight-line V fitted to the visible shirt).
All other parts keep their traced, detail-preserving outlines.
