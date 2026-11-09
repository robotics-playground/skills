# Research behind the method

What the literature says, and what this skill takes from each. Read when changing the pipeline.

| Work | Key idea | Used here as |
|---|---|---|
| **Perception-Driven Semi-Structured Boundary Vectorization** - Hoshyari, Dominici, Sheffer, Carr, Ceylan, Wang, Shen, SIGGRAPH 2018 ([project](https://www.cs.ubc.ca/labs/imager/tr/2018/PerceptionDrivenVectorization/), [Adobe](https://research.adobe.com/publication/perception-driven-semi-structured-boundary-vectorization)) | Viewers expect corners only at real boundary discontinuities; between corners, the simplest continuous curves that capture regularities. Corner detection = learned perceptual metric + discontinuity analysis. | `contours.real_corners` (straight + non-bending sides) and `regularize` (simplest of line < arc < line/arc pieces < spline within tolerance). Would have pre-empted ~13 of 27 review rounds (sharp corners, continuous curves, straight V, circular chin, small bulge). |
| **ShipShape: a drawing beautification assistant** - Fišer, Asente, Sýkora, SBIM 2015; extended in Computers & Graphics 2016 ([EG](https://diglib.eg.org/handle/10.2312/exp20151178), [project](https://dcgi.fel.cvut.cz/~sykorad/shipshape.html)) | Snap strokes to geometric relations: endpoint snapping, tangent alignment, parallel/perpendicular lines, equal lengths, arc-centre snapping, reflection symmetry. | Regularity snapping after fitting: tangent-aligned line/arc pieces, `regularize.mirror_pairs` for symmetry candidates, shared polygons between adjacent parts (jacket V = cravat V). |
| **Differentiable Vector Graphics Rasterization (diffvg)** - Li, Lukáč, Gharbi, Ragan-Kelley, SIGGRAPH Asia 2020 ([project](https://people.csail.mit.edu/tzumao/diffvg)) | Rasterization with anti-aliasing prefiltering is differentiable, so vector parameters can be optimized against the raster directly. | Future: fit primitives/curves against the anti-aliased artwork pixels instead of a thresholded mask (would replace unmix + trace for clean flat art). Not used yet (needs PyTorch + diffvg build). |
| **LIVE: Towards Layer-wise Image Vectorization** - Ma et al., CVPR 2022 ([project](https://ma-xu.github.io/LIVE)) | Add closed Bézier paths layer by layer; compact, semantically consistent layered output. | Supports the layered model: parts as closed paths stacked back-to-front, hidden portions completed. |
| **LayerPeeler** (arXiv 2505.23740, 2025) | VLM builds a layer graph of occlusions, then removes top layers to recover hidden regions. | Same idea as the part spec's stacking order + hidden completion; a VLM-built occlusion graph is a candidate for automating step 1. |
| **CADCodeVerify / Seek-CAD / Ortho2CAD / CADSmith / Honda multi-agent CAD (2024-2026)** | Generate -> render -> compare -> refine; pure visual critique misses dimensional errors, so add numeric checks. | Self-review before any human review: `checks` (scope guard, overlap under front parts, gaps/slivers, thin features, same-colour neighbours) + the visual question list. |
| **IntroSVG** (CVPR 2026), **Render-in-the-Loop** (arXiv 2604.20730) | Generator-critic loops on rendered SVG; conditioning on intermediate renders. | Supports the render-and-critique step; evidence on reducing *human* iterations is still thin, so keep the numbered review sheet as the human checkpoint. |

## Takeaways that changed the skill
1. **Regularize automatically, then ask once.** Edge regularity (corner / line / arc / smooth curve) is a
   perceptual problem with a known solution; do it before the user sees anything, and present the
   choices on a numbered sheet.
2. **Two tolerances.** Fidelity (do not move an edge more than the image's precision: 1/4 effective px)
   and regularity (accept a simpler curve within 1/2 effective px), plus a coherent-feature guard so a
   1 px drawn bump is not flattened.
3. **Layers are first-class.** Occlusion order, hidden completion and minimum overlap under front parts
   are part of the model, not post-fixes.
4. **Numbers + pictures.** Every self-check has a number (deviation, overlap, area) and a zoomed image.
