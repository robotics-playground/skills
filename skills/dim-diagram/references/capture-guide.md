# Capture Guide (phone-first)

**Accuracy is won here, not in software.** Target ±1–2 %. The error budget below is dominated
by capture geometry; the algorithm is secondary.

---

## 0. One-time setup

1. **Print the board.** Print `assets/charuco_A4.pdf` **at 100 % / Actual size** (no "fit to
   page"). **Measure the printed "100.0 mm" line with a ruler.** If it isn't 100.0 mm, the
   printer scaled it — reprint, or pass the *measured* square size to the pipeline. Print
   scaling is a direct multiplier on every output dimension. Glue the print to rigid flat
   backing (foam board / clipboard); a bowed board adds error.
2. **(Recommended) calibrate your phone lens** once and lock that lens (see
   `rectification.md`). Skipping it costs ~1 % at the frame edges.

---

## 1. Board placement — beside the object, coplanar with the measured face

**The board goes *beside* the object, never under it**, lying in the *same flat plane* as the
face being measured and parallel to the camera. Both the full object silhouette and the full
board must be visible. **Orientation changes per view:**

| View | Camera looks | Measured plane | Board placement |
|------|--------------|----------------|-----------------|
| **Front** | horizontally at the front | vertical (width × height) | **stood up vertically** beside the object, facing the camera |
| **Side** | horizontally at the side | vertical (depth × height) | **stood up vertically** beside the object |
| **Top** | straight down | horizontal footprint (width × depth) | **laid flat** on the table beside the object |

Bird's-eye view of the **front** setup (board is a standing panel next to the object, both at
the same distance from the camera):

```
   board (standing,      object
   facing camera)        ┌──────┐
      ▓▓▓▓                │      │
   ────┴─────────────────┴──────┴────  table
                  ▲
               [phone]  far away, square-on
```

**Why not on/under the object:**
1. **Plane mismatch (dominant).** Front/side faces are vertical; a flat-lying board is
   horizontal (90° off) so its homography cannot rectify that face.
2. **Occlusion.** An object on the board hides center squares.
3. **Depth offset.** An object resting on the board lifts the silhouette above the board plane
   → error ≈ Δ/d.

**Tall-object top-view nuance:** if the widest part is elevated above the table, **raise the
flat board to that height** (prop it on a box) so it stays coplanar with the silhouette.

---

## 2. The error budget (ordered by leverage)

| Source | If ignored | Target | Mitigation |
|--------|-----------|--------|------------|
| **Coplanarity (Δ offset)** | **≈ Δ/d** | Δ < 1 % of d | board in the measured plane / mid-depth |
| Perspective/depth | ≈ depth/distance | distance ≥ 10× depth | shoot far + long lens |
| Lens distortion | ~1 % (edges) | reproj < 0.3 px | calibrate + undistort; object off corners |
| Tilt/keystone | ∝ tan(tilt)·extent/d | tilt < 2–3° | tripod + level; pipeline warns past ~2.5° |
| Pixel noise | sub-pixel | marker 15 px–0.5 % frame | high-res, matte, diffuse light, multi-shot |

Quantified example of the #1 lever: a reference plane 8 cm in front of the object face, shot
from 80 cm → **10 % error**. The same plane at 400 cm → 2 %. Coplanar placement removes it.

---

## 3. Per-view capture checklist

For each of front / top / side:

1. Matte, **high-contrast background** (dark object → white paper; light object → dark mat).
2. **Board placed per §1** — beside the object, coplanar with the face.
3. **Tripod, square to the face** (use the camera-app level/grid), then **step back far**
   (≥ 10× the object's front-to-back depth).
4. **Pick the right lens** (see §4).
5. **Lock AE/AF**, max resolution, **HDR / Live / Portrait / Beauty / flash OFF**, diffuse even
   light, no glare/hard shadows.
6. Fill the frame but keep the object and board **clear of the extreme corners**.
7. **(Recommended) add a validation object** of known length (≥ 50 % of the object size); its
   measured residual is your honest per-shot accuracy readout.
8. Shoot **3–5 frames**, keep the sharpest.

---

## 4. Phone do / don't

| Do | Don't | Why |
|----|-------|-----|
| Main **1×** lens, or optical **2×/3×** tele | **Ultra-wide 0.5×** | most distorted lens |
| **Move your feet** to frame | Pinch / digital zoom | upscaled crop, fake pixels |
| Use **one calibrated lens** | Switch lenses mid-session | per-lens intrinsics |
| **Lock focus** before shooting | Let it refocus per shot | focus "breathing" shifts focal length |
| RAW / Pro mode if available | Scene-optimizer / AI enhance | computational warping moves edges |

The optical telephoto is ideal if your room is big enough: it lets you stand far (low
perspective error) while filling the frame.

---

## 5. The three views and how they cross-check

Each photo captures two of the three world axes — width (X), depth (Y), height (Z):

| View | Captures | Shares with |
|------|----------|-------------|
| **Front** | width X × height Z | width ↔ Top, height ↔ Side |
| **Side** | depth Y × height Z | height ↔ Front, depth ↔ Top |
| **Top** | width X × depth Y | width ↔ Front, depth ↔ Side |

Width must read the same in Front and Top; height in Front and Side; depth in Top and Side. The
pipeline reconciles these and **flags any view whose shared dimensions disagree** — an automatic
"you mis-captured this view" detector. Shooting straight down for the top view is the awkward
one: use an overhead mount or carefully hold the phone parallel to the table using the level.

---

## 6. Honest limits

±1–2 % is realistic; ~0.5–1 % with everything above done well. **Sub-mm / <0.5 % is not
achievable from single photos** — use the `photogrammetry-3d` sibling skill (multi-photo) or
calipers. Two residual limits no software fixes: the silhouette of a rounded/3D object slightly
over-measures its true width, and a single view can't see occluded/internal features.
