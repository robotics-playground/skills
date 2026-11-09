# Working with the user (review loop)

## What the user sends
- Screenshots of the drawing with **green boxes** or **numbered red markers** on problem spots, and a
  short instruction per mark ("(1) sharp, (2)(3) continuous curve", "add a small curve here like the
  original", "smooth curve and remove the gap").
- Short corrections ("No, ...", "The top is smaller", "Like the original image").
- New reference images mid-task ("use this for how it looks in real life").

## Aim: at most 3 user rounds
Round 1 = spec + questions, round 2 = one numbered edge sheet, round 3 = touch-ups. If you are about to
send a fourth review request, stop and ask yourself which self-check or rule should have caught it, and
add it to `checks.py` / the construction rules.

## How to respond each round
1. Restate in one line what will change and **what will not**.
2. Locate the feature in the pixels: zoom the artwork (nearest neighbour) with the current outline.
   Measure before changing (profiles, edge points, widths).
3. Change only that feature; keep its neighbours fixed (pinned spline ends, joins at equal width,
   shared polygons between adjacent parts).
4. Verify with a pixel overlay; also check the other side for symmetric features.
5. Diff all untouched parts against the previous round, and the edited part outside the asked zone
   (`checks.changed_outside`); nothing else may move. When a run must be refit, splice (method 9a).
6. Re-export, republish the review page to the same URL, send the zoomed before/after.
7. Report briefly: what changed (in mm), the largest shift and where, anything hidden by another part.

## Vocabulary map (user words -> geometry)
| User says | Means |
|---|---|
| "sharp corner" | vertex at the intersection of two fitted side lines/tangents |
| "continuous curve" | one spline / one analytic curve over the whole edge, ends pinned |
| "straight lines" | polygon edges fitted to the pixels |
| "small curve like the original" | keep a measured small feature (bulge/notch) with its true depth |
| "1 piece" | merge parts, remove their seam, keep each side's edge rules |
| "made from two ovals" | ask: one ring (outer - inner) or two loops? |
| "bigger, X on top of it" | the part continues under X; only the hidden portion grows |
| "hidden part / visible part" | different rules: hidden = anatomy/mechanics, visible = image |
| "not mapping good enough" + a crop of a small area | the small drawn features there (1-4 px steps) are lost; measure them on the pixels (method 7e) |
| "you did not capture the sharp corner" | square corners on short steps; zoom 16-20x with the pixel grid before answering |
| "should not be a continuous curve, two X touching" | one curve per shape, sharp notch where they meet |
| "not symmetry" / "the face is slightly turned" | keep those details as drawn (raw masks); others stay mirrored |
| "connect X to Y" (moving parts) | X ends exactly on Y's edge; no part runs on behind Y |
| "is a straight line, no need arc" | change the edge type only (stroke/run), keep its measured position and width |

## Things that broke trust
- Changing an approved part while fixing another.
- Building one interpretation of an ambiguous request without asking.
- Over-smoothing real detail and calling it "defect removal".
