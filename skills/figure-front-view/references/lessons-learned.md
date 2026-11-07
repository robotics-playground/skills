# Lessons learned - Keep / Problems / Try next time

Source project: "Helmet Guy" squat figure (Vinc Studio design, blue helmet, sunglasses, black suit,
oval arm ring, white cravat and hands, black boots, rectangular base). Inputs: one straight-on
product-sheet artwork (~240 px wide figure), two phone photos of the 3D-printed toy. About 30 review
rounds with the user. Final: 12 parts, figure 99.9 mm, base 70 x 20.3 mm, outline IoU 0.98 vs artwork.

The timeline is condensed here so the next project can skip the detours.

---

## 1. Timeline of the user's corrections (what they actually wanted)

| # | User said | What I had done | What was really wanted |
|---|---|---|---|
| 1 | "research the best way first; the agent must understand the objects, not just the image" | Traced the photo directly | Understand -> spec -> model -> verify (image is evidence, not truth) |
| 2 | "shape detail, pose and face are important. Don't remove them" | Replaced every part by a fitted primitive | Primitives only for structure; visible edges traced |
| 3 | "only ignore the defect by raster image" | Thresholds + smoothing created/kept fringes and bumps | Remove only raster defects |
| 4 | "corner needs more sharpness; bottom edge is not a continuous curve - two fists touch" | 1.2 px smoothing erased the fist notch and thumb corners | Keep features ~1 px; rebuild corners |
| 5 | "calculate how much width of the pixel ... adjust the raster defect ratio" | Hand-picked tolerances | Measure effective pixel + noise, derive the budget |
| 6 | "two arms made from two ovals" | One ring -> I split it into two mirrored loops | (see #7) |
| 7 | "No, ... two ovals, 1 inside 1 outside, not separate into two parts" | Two loops | ONE ring = outer oval minus inner oval |
| 8 | "use this (photo) for how it looks in real life" | Artwork only | Print photo decides smoothness, construction, layering |
| 9 | "Cravat: V shape with straight lines" / "The top is smaller" | Traced drop shape / parallel sides | Straight-line V; sides taper wider downward |
| 10 | "pants and jacket should be 1 piece" | Separate parts | SUIT = jacket + pants |
| 11 | Boxes on screenshot: "smooth curve and remove the gap" / "corner more sharp" | Waist step, wavy legs / rounded fill | Gap filled + **sharp** waist corner from the hip tangent |
| 12 | Numbered marks: "(1) sharp, (2)(3) continuous curve" | Wavy traced legs | Gap = 3 fitted curves meeting at sharp corners; outer/inner leg edges = one spline each |
| 13 | "add a small curve here like the original" | Quadratic flattened the crotch bulge | Flat edge + small round-bottomed bulge (0.35 mm) |
| 14 | "face is bigger in the real print; helmet on top" | Face traced only where visible | Face piece continues under the helmet |
| 15 | "not just a circle; hidden part normal face mechanic; visible part follow the image" | Circle, then photo-fitted jaw (changed visible part) | Hidden = normal face shape; visible = image |
| 16 | "bottom border of the face is circle ... like the original image" | Traced jaw | Visible chin = circular arc |
| 17 | "line at the chin needs to be higher" | Arc pulled down by white shirt pixels | Fit the face/jacket edge only, excluding shirt columns |
| 18 | "I told you fix the chin. Why change the top of the head?" | I also "improved" the hidden forehead | Change only the chin |

Pattern: the user thinks in **edges and features** (this corner sharp, this edge one curve, this part
one piece, this bulge like the original). The pipeline must let you change ONE edge of ONE part
without touching anything else.

---

## 2. KEEP (worked, reuse as-is)

**Process**
- Understand first: a written part spec (parts, colour, primitive, relations, stacking order). The
  user corrected structure early because it was written down.
- Authority per question: artwork for shape intent; print photo for construction/smoothness/hidden
  sizes; measurement for scale. The print photo resolved many disputes in one look (arms smooth,
  jacket straight, seams show the arm ring in front of the jacket, shirt behind jacket, face under helmet).
- A published review page (Artifact) with: overlay slider of rebuilt parts on the artwork pixels,
  dimensioned front view, parts sheet with per-part size/colour/max-shift, method, open questions.
  Republish to the same URL each round.
- Pixel-level zoom overlays (nearest-neighbour upscaled crop + red outline) as the verification
  tool for every change. They found every bug that the IoU numbers missed.
- Status updates in plain words while working; ask for the one number you can't see.

**Techniques** (all in `pipeline/figfront/`)
- Anti-alias **colour unmixing** with a flat-paint palette (pixel = paint or 2-paint blend; label
  by alpha >= 0.5). Removed fringes, the false "blue" rim at black|background blends, and bumps.
- **Pixel calibration**: edge 10-90 % width vs simulated ideal edge -> effective pixel (1.33 px =
  0.42 mm here); noise 0.026 px. Budget: max shift 1/4 effective px (0.105 mm), sigma 1/8, corner
  scale 1 effective px.
- **Partition by primitives, edges by trace**: same-coloured parts split by fitted primitives
  (front-most wins), visible boundary from pixels.
- **Hidden-part completion**: part = visible OR (primitive OR visible grown 3 px) where a front part
  covers it. Prevents slivers/white gaps.
- **Round (disk) morphology** only, and seam welding only next to front parts.
- **Real-corner rebuild**: corner = straight sides that keep direction AND meet at >= 32 deg; vertex
  = intersection of the side lines (moves <= 1 effective px).
- **Exact geometry where the print is geometric**: oval ring arms (analytic), straight-sided jacket
  with the cravat V cut out (shared edge -> no gap), straight-line tapered V cravat, gap between legs
  = 3 fitted curves with sharp corners, outer leg edges = pinned LSQ spline, waist corner from the hip
  tangent meeting the jacket line, circular chin measured on the edge pixels.
- **Visible regions by subtraction** in stacking order -> gap-free multi-colour print regions.
- DXF as true SPLINEs split at real corners, one layer per part; a second DXF of visible regions.

---

## 3. PROBLEMS (and their root cause)

| Problem | Root cause | Fix used | Better next time |
|---|---|---|---|
| Lost pose/face detail | Primitive replaced visible edges | Primitives only for partition/hidden | Start with this rule |
| Bumpy edges, coloured fringes, false third colour | HSV thresholds | Palette unmixing | Start with unmixing |
| Real notch/corners smoothed away | Tolerance guessed (1.2 px) | Calibrated budget (0.33 px) | Calibrate before any smoothing |
| Facets on curves -> fake corners | scipy diamond morphology | `disk_op` | Ban `iterations=` morphology |
| Spikes on round thumb tips | Corner = high local turning | Straight + non-bending + side-angle test | Use `real_corners` |
| Compression specks grown into bumps | Leftover pixels attached to nearest part at the figure edge | Never assign leftovers within 1 px of background | Same |
| Base top edge glued to boots | Sole line threshold | Cut at the sole line | Detect base top explicitly |
| Ring sliver between face and helmet | Head-zone leftovers assigned to body parts | Head zone: only head parts | Same |
| Two-oval misread | Ambiguous phrase, I picked one meaning | Asked / reverted | Sketch both options, ask before building |
| Chin arc too low | White shirt pixels fitted as face | Fit the face/jacket edge, exclude shirt columns | Check same-colour neighbours before any fit |
| Changed the forehead while fixing the chin | Scope creep ("improve while here") | Reverted | Diff untouched parts automatically before publishing |
| `smooth.py` grew to many per-part special cases | Each review added a patch in one script | - | Per-part **edge spec** in config (see below) |
| Hard-coded crop/rows/columns | One-off scripts | - | Config file per toy; features found from parts, not coordinates |
| Scale chain is weak | Base width 70 mm guessed from a thumbnail; figure height from that | Accepted by user | Ask for one caliper measurement up front |
| Artwork vs print disagree (head size, arm shape, legs 1 mm off-axis) | Illustration != product | User chose artwork; photo for construction | Make the authority table explicit at step 2 |
| Thin features (0.5 mm mouth line) unprintable | Character art at toy scale | Flagged, not changed | Ask early how face lines will be made (inlay / raised / engraved) |

---

## 4. TRY NEXT TIME

1. **Edge spec per part, not just a part spec.** Every boundary segment gets a type:
   `traced | straight | arc(circle) | oval | spline(continuous) | bulge | corner(sharp|round)` plus its
   source (artwork/photo/exact). The user's feedback maps 1:1 onto this table, and a change touches
   one row. Generate geometry from the table; keep the trace as the fallback type.
2. **Config-driven pipeline** (`toy.yaml`): palette points, crop, scale reference, part list,
   stacking order, edge spec, authority table. The scripts in `examples/helmet-guy/` should become
   `figfront` commands that read it.
3. **Calibrate first, always**: run `calibrate.metrics` right after unmixing and print the budget to
   the user in mm.
4. **Look at the print photo at step 1**, not step 20: seams -> stacking order and piece count;
   smooth/straight edges -> which parts get exact geometry; hidden sizes (face under helmet).
5. **Numbered feature markers on the review page** (1, 2, 3 ... on corners/edges) so the user can
   say "make 4 sharp" without drawing boxes. Optionally the page's comment capability.
6. **Untouched-part diff guard**: before publishing, compare every part not named in the request
   with the previous version (Hausdorff distance); if any moved > 0.01 mm, stop and explain.
7. **Ambiguity protocol**: when a request admits two geometries, render both small and ask (one
   message), instead of building one and being corrected.
8. **Same-colour neighbour check** before every fit: list adjacent parts with the same paint and
   exclude their columns/regions (shirt under chin, face vs hands, black parts).
9. **Ask for real dimensions early**: one caliper value, plate thickness, slot depth, how face lines
   are printed. Then go to 3D: extrude visible regions per colour, arm ring as a separate rotating
   body with clearance and pivot, tab + slot to the base.
10. **Second artwork variant check**: the product sheet had 6 variants sharing the body; keep the
    body/accessory split so a new variant only re-fits the head/accessories.
