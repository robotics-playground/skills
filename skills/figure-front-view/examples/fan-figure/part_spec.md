# Part specification - fan figure (Vinc Studio "helmet guy", squat pose)

Personal use only. Character design belongs to Vinc Studio. Do not publish artwork or derived outlines.

## Object understanding
- Type: desk toy, FDM print, 3 colours (blue, white, black).
- Construction: flat plates of constant thickness, colours side by side. Figure stands in a rectangular base.
- Moving part: ARMS ring (with HANDS) turns 360 deg about an axis normal to the plate, at the chest
  (video `t00`-`t14`). Head plate is in front of the ring. Body plate is behind the ring.
- Consequence of the motion: every body area that the ring covers at rest shows when the ring turns.
  Thus hidden body outlines are real outlines: jacket = straight panel from nose level down
  (video `t03`, `t12` show it uncovered).
- Symmetry: one mirror axis for the whole figure. Body, arms, hands, helmet and face outline are exactly symmetric. Face details (glasses, brows, nose, mouth) follow the artwork as drawn: the face is slightly turned.
  The artwork draws the legs 1.04 mm left of the body axis; the model moves them onto the axis.

## Sources and authority
| Question | Source that wins | Notes |
|---|---|---|
| Visible outline shape, proportions | artwork (`input/2026-10-31 19_59_59-_ fan-figure - FreeCAD 1.1.4.png`) | 246 x 385 px, legs moved onto axis |
| One piece or several, stacking order | print photo `2cf1e25b-...jpg` + video frames | seams, ring motion |
| Smooth / straight / circular edges | print photo | ring smooth oval, jacket straight, cravat straight V, cravat plain white (no tie) |
| Hidden portions | video (ring turns) + anatomy | jacket panel, full head under helmet |
| Scale | NOT MEASURED | provisional: figure height 99.9 mm (value used for the same figure in the skill example) |

## Pixel calibration
| Value | Result |
|---|---|
| Nominal pixel | 0.339 mm/px (from provisional figure height 294.5 px = 99.9 mm) |
| Edge spread 10-90 % | 1.70 px (305 samples, helmet and legs against background) |
| Effective pixel | 1.50 px = 0.51 mm |
| Edge noise | 0.03 px rms |
| Max shift (fidelity tolerance, 1/4 eff px) | 0.375 px = 0.127 mm |
| Regularity tolerance (1/2 eff px) | 0.75 px = 0.25 mm |

## Parts
| # | Part | Colour | Construction | Outline rule | Relations |
|---|---|---|---|---|---|
| 1 | BODY | black | jacket + pants + boots = one plate | jacket: exact rectangle 29.4 mm wide, top at nose level; V-neck = CRAVAT polygon (shared edge); hips, legs, crotch gap, boots: traced, symmetric, lines + tangent arcs; sharp waist corners | behind ARMS, HANDS, FACE |
| 2 | CRAVAT | white | straight-line V | exact: 3.49 mm wide at chin, 4.59 mm at V start, V 4.56 mm deep, sharp tip; top continues under the face to nose level | behind BODY, visible in V-neck |
| 3 | ARMS | black | one arm loop, ends on the hand sides (edge 12) | visible edges: exact ellipses, outer 60.40 x 35.62 mm, inner 37.06 x 21.34 mm, inner centre 0.83 mm lower; inside the jacket width (14.70 mm from axis) each edge turns tangentially into the end of the hand side, max 0.51 mm off the oval | turns about chest axis with HANDS; behind head plate, in front of BODY |
| 4 | HANDS | white | two fists touching, 2 thumbs | traced, symmetric, lines + tangent arcs; bottom = two arcs R 15.08 mm (one per fist), sharp notch on the axis | closes the arm loop between the arm ends, turns with ARMS |
| 5 | FACE | white | full head plate | visible: helmet opening + chin arc R 17.23 mm; hidden: superellipse head, reaches 2.0 mm under helmet, kept 1 mm inside helmet outline | behind HELMET, in front of ARMS |
| 6 | HELMET | blue | dome + 2 flaps | traced, symmetric, tangent arcs | in front of FACE |
| 7 | GLASSES | black | 2 lenses + bridge | traced as drawn (not mirrored) | face inlay |
| 8 | EYEBROWS | black | 2 arc strokes | as drawn: left width 0.84 mm, right 0.70 mm; round ends | face inlay |
| 9 | NOSE | black | dot | traced as drawn | face inlay |
| 10 | MOUTH | black | upper lip straight stroke + lower lip arc stroke | as drawn: upper lip straight (slight tilt), width 0.70 mm; lower lip arc, width 0.60 mm; round ends | face inlay |
| 11 | BASE | black | rectangle | 76.66 x 23.29 mm, centred on figure axis (artwork draws it 2.71 mm left) | BODY stands on top |

## Edge spec (numbers = `edge_sheet.json`, right half; left half is the mirror)
| Part | Edge | Type | Source |
|---|---|---|---|
| HELMET | dome, flaps, opening | one smooth curve (tangent arcs) | trace |
| FACE | chin | one arc R 17.23 mm, centre on axis | face/jacket edge, shirt columns excluded |
| BODY | jacket sides, jacket top | straight | exact |
| BODY | waist corner | sharp | jacket line meets traced hip |
| BODY | hip + outer leg (waist to heel notch) | one smooth curve | trace |
| BODY | crotch top | flat with 0.4 mm round dip at centre | trace |
| BODY | boots | square steps, sharp corners: inner ledge + 3 vertical steps; outer ledge, notch, slanted step; curved toe; straight sole | 50 % edge crossings on both boots, averaged |
| BODY | inner leg | smooth curve | trace |
| CRAVAT | sides, V edges | straight, sharp tip | exact |
| ARMS | outer, inner (visible) | ellipse | exact |
| ARMS | arm ends | tangent arcs into the hand side corners | construction (artwork black on black) |
| HANDS | sides, top between thumbs | straight | trace (regularized) |
| HANDS | bottom | two fist arcs R 15.08 mm, sharp notch on axis | column profile, both fists averaged |

## Stacking order (back -> front)
BASE -> CRAVAT -> BODY -> ARMS -> HANDS -> FACE -> HELMET -> GLASSES / EYEBROWS / NOSE / MOUTH

## Open questions for the user
1. One real dimension (caliper): figure height, base width, or helmet width.
2. Plate thickness, and how the face lines are made (inlay / raised / engraved). Brows 0.68 mm and
   mouth 0.55-0.70 mm are below 0.8 mm (2 lines of a 0.4 mm nozzle).
3. Symmetry: legs moved 1.04 mm onto the body axis, base centred (artwork: 2.71 mm left). Keep?
4. Crotch top: keeps its 0.4 mm dip. Keep it?
5. Waist corner: jacket side meets the hip with two breaks (18 deg and 29 deg) over 1 mm. Make it one sharp corner?
