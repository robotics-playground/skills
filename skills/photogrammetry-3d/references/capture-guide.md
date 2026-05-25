# Capture Guide: orbit & turntable

You capture **two things for the same object**: (a) the **3 orthographic shots** for
`dim-diagram` (per its capture guide — they yield the metric bundle), and (b) the **overlapping
photo set** for reconstruction here. The 3 orthographic views may be reused inside the set.

## Common requirements (both methods)

- **Surfaces:** matte/textured surfaces reconstruct well. **Shiny / transparent / featureless
  surfaces fail SfM** — dull them with chalk spray / AESUB / powder if acceptable, else expect
  holes (a real risk for glossy robot parts).
- **Markers visible in many frames** — the shared ChArUco board is the registration anchor that
  ties the photo set to the bundle's metric frame. Keep it in view.
- **Lighting:** even, diffuse, no moving shadows, no specular glare.
- **No motion blur, max resolution, lock focus & zoom**, don't change lenses mid-capture.
- **Don't move the object** between shots (move the camera instead — except turntable).
- Include a **known-length validation object** for an independent accuracy readout.

## Handheld orbit

Walk a full circle around the object at **2–3 heights** (low / eye / high), shooting every
~10–15° with **60–80 % overlap** — roughly **20–60 photos**. Keep the object and markers fully
in frame. Most flexible; no rig needed.

## Turntable

Camera fixed on a tripod; object + markers sit on the turntable and rotate in ~10–15° steps
(24–36 shots) at **2–3 camera elevations**.

**Critical caveat:** the object moves relative to a *static* background, which corrupts SfM
(it will try to reconstruct the room, not the object). Use a **featureless/uniform background
or mask the object** (the `ingest` step supports masks). The markers ride the turntable with
the object, which is exactly what registration needs.

## Trade-offs

| | Rig | Background | Lighting consistency | Effort |
|---|---|---|---|---|
| **Orbit** | none | tolerant | varies as you move | low setup, more shots |
| **Turntable** | tripod + turntable | must mask / be featureless | very consistent | more setup, repeatable |
