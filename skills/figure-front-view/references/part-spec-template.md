# Part spec template

Copy into the project as `part_spec.md`, fill it BEFORE fitting, show it to the user, and keep it
updated after every review round. A complete filled example: `examples/helmet-guy/part_spec.md`.

```markdown
# Part specification - <figure name> (<designer / source>)

## Object understanding
- Type: <desk figure / standee / keychain>, <FDM multi-colour / acrylic / layered>, colours: <...>
- Construction: <constant-thickness plate in a base slot / layered plates / ...>
- Moving parts: <e.g. arm ring rotates (sheet panel 6)>
- Symmetry rule: <mirror about one axis? known intentional asymmetries>

## Sources and authority
| Question | Source that wins | Notes |
|---|---|---|
| Shape / proportions of visible outlines | artwork | |
| Construction: one piece or several, what is in front | print photo seams | |
| Smooth / straight / circular edges | print photo | exact geometry only where seen |
| Size of hidden portions | print photo + anatomy/mechanics | |
| Scale | measurement: <value, how measured> | |

## Pixel calibration (fill after measure)
nominal mm/px, edge spread, effective pixel, noise, max shift (mm), corner scale

## Parts
| # | Part | Colour | Primitive (structure only) | Outline rule | Relations |
|---|---|---|---|---|---|
| 1 | HELMET | blue | ... | traced | on top of FACE |
| 2 | FACE | white | ... | visible: <traced / circle chin>; hidden: normal face shape | behind HELMET |
| ... | | | | | |

## Edge spec (one row per boundary segment the user may talk about)
| Part | Edge | Type | Source | Notes |
|---|---|---|---|---|
| SUIT | waist corner L/R | corner (sharp) | hip tangent x jacket line | |
| SUIT | outer leg edges | spline (continuous) | trace, pinned ends | |
| SUIT | gap top | flat + bulge | trace measure | depth 0.35 mm |
| CRAVAT | sides | straight (tapered) | fit to visible white | |

## Stacking order (back -> front)
<... -> ... -> ...>  (from seams on the print)

## Open questions for the user
- one real dimension, plate thickness, slot depth, how thin face lines are printed
```
