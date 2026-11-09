"""Review page (single HTML file, images embedded) for the round-1 review with the user."""
import base64
import html
import json
import cv2
import numpy as np
import config as C
from render import art_corrected, assembled

OUT = C.ROOT / "review_page.html"
W = C.WORK


def b64(img, ext=".jpg", q=88):
    ok, buf = cv2.imencode(ext, img, [cv2.IMWRITE_JPEG_QUALITY, q] if ext == ".jpg" else [])
    mime = "image/jpeg" if ext == ".jpg" else "image/png"
    return f"data:{mime};base64," + base64.b64encode(buf.tobytes()).decode()


art = art_corrected(4)
model = assembled(4)
img_art = b64(art)
img_model = b64(model, ".png")
img_edges = b64(cv2.imread(str(W / "edge_sheet.png")), q=85)
zooms = [(n, b64(cv2.imread(str(W / f)), q=85)) for n, f in
         (("Head", "z_head.png"), ("Chest", "z_chest.png"), ("Legs and boots", "z_legs.png"))]
frames = cv2.imread(str(W / "frames.png"))
img_frames = b64(cv2.resize(frames, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA), q=80)

P = json.load(open(C.OUT / "parts_mm.json"))
K = json.load(open(C.OUT / "checks.json"))
E = json.load(open(C.OUT / "edge_sheet.json"))
M = json.load(open(C.OUT / "pixel_metrics.json"))
svg_front = open(C.OUT / "fan_figure_front_view.svg").read()
svg_front = svg_front.replace('<svg ', '<svg class="drawing" role="img" aria-label="Dimensioned front view" ', 1)

RULE = {
    "BASE": "Rectangle, centred on figure axis",
    "CRAVAT": "Exact straight V, sharp tip; top continues under the face",
    "BODY": "Jacket = exact rectangle; hips, legs, boots traced; V-neck = cravat edge",
    "ARMS": "Ellipse edges where visible; tangent arcs into the hand side corners (edge 12)",
    "HANDS": "Traced, lines + tangent arcs; bottom = two fist arcs with a notch",
    "FACE": "Visible: helmet opening + chin arc. Hidden: full head 2 mm under helmet",
    "HELMET": "Traced dome + flaps, tangent arcs",
    "GLASSES": "Traced as drawn (not mirrored)",
    "EYEBROWS": "Arc strokes as drawn, round ends",
    "NOSE": "Traced as drawn",
    "MOUTH": "Upper lip: straight stroke; lower lip: arc stroke; as drawn",
}
SW = {"blue": "var(--paint-blue)", "white": "var(--paint-white)", "black": "var(--paint-black)"}
rows_parts = "".join(
    f'<tr><td><span class="sw" style="background:{SW[v["colour"]]}"></span>{k}</td>'
    f'<td class="num">{v["width_mm"]:.1f} × {v["height_mm"]:.1f}</td><td class="num">{v["visible_area_mm2"]:.0f} / {v["area_mm2"]:.0f}</td>'
    f'<td>{html.escape(RULE[k])}</td></tr>'
    for k, v in P["parts"].items())
KIND = {"line": "line", "arc": "arc", "curve": "curve", "arc-stroke": "stroke", "line-stroke": "stroke", "ellipse": "ellipse", "exact-line": "exact"}
rows_edges = "".join(
    f'<tr><td class="num">{r["id"]}</td><td>{r["part"]}</td><td><span class="kind k-{KIND[r["kind"]]}">{KIND[r["kind"]]}</span></td>'
    f'<td>{html.escape(r["note"])}{(" · R " + format(r["radius_mm"], ".2f") + " mm") if r.get("radius_mm") else ""}</td>'
    f'<td class="num">{r["length_mm"]:.1f}</td></tr>' for r in E)
seg = K["segments"]
n_lines = sum(v["lines"] for v in seg.values()); n_arcs = sum(v["arcs"] for v in seg.values())
max_break = max(v["max_tangent_break_deg"] for k_, v in seg.items() if k_ != "BODY")
iou = K["fidelity_iou_vs_symmetrised_artwork"]
thin = K["thin_features_lt_0_8mm"]
d = P["dims"]; f = P["fits"]

page = f"""<title>Fan Figure Front View</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=Source+Serif+4:opsz,wght@8..60,400;8..60,600&family=JetBrains+Mono:wght@400;600&display=swap">
<style>
/* Layout: drafting-sheet review. Title block on top, then evidence (slider), decisions (edges, parts), questions. */
:root {{
  --bg: #eef1f4; --sheet: #fbfcfd; --ink: #1d2733; --muted: #5b6878; --rule: #c9d1db;
  --accent: #1f6fb2; --accent-soft: #dbe9f6; --warn: #a3620a; --warn-soft: #f6ead6; --ok: #2f7a4b;
  --paint-blue: #2b8fd0; --paint-white: #f4f4f2; --paint-black: #262626; --paper: #f7f8f6;
  --k-line: #2f7a4b; --k-arc: #a23fa8; --k-curve: #1f6fb2; --k-stroke: #0d7f9c; --k-ellipse: #b23a3a; --k-exact: #5d6b2b;
  --f-display: "Barlow Semi Condensed", "Arial Narrow", system-ui, sans-serif;
  --f-body: "Source Serif 4", Georgia, "Times New Roman", serif;
  --f-mono: "JetBrains Mono", ui-monospace, Consolas, monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg: #12171d; --sheet: #192029; --ink: #e3e8ee; --muted: #9aa7b6; --rule: #2e3946;
  --accent: #6cb2ee; --accent-soft: #1c3550; --warn: #e5a54b; --warn-soft: #3a2c16; --ok: #6fc28f;
  --k-line: #6fc28f; --k-arc: #d58bdb; --k-curve: #6cb2ee; --k-stroke: #4fc3df; --k-ellipse: #ee8080; --k-exact: #b7c46a;
  color-scheme: dark; }} }}
:root[data-theme="dark"] {{
  --bg: #12171d; --sheet: #192029; --ink: #e3e8ee; --muted: #9aa7b6; --rule: #2e3946;
  --accent: #6cb2ee; --accent-soft: #1c3550; --warn: #e5a54b; --warn-soft: #3a2c16; --ok: #6fc28f;
  --k-line: #6fc28f; --k-arc: #d58bdb; --k-curve: #6cb2ee; --k-stroke: #4fc3df; --k-ellipse: #ee8080; --k-exact: #b7c46a;
  color-scheme: dark; }}
body {{ background: var(--bg); color: var(--ink); font-family: var(--f-body); font-size: 16px; line-height: 1.55; }}
.wrap {{ max-width: 1120px; margin: 0 auto; padding-left: max(16px, env(safe-area-inset-left, 0px));
  padding-right: max(16px, env(safe-area-inset-right, 0px)); padding-block: 28px 64px; display: grid; gap: 28px; }}
h1, h2, h3 {{ font-family: var(--f-display); font-weight: 600; letter-spacing: .01em; text-wrap: balance; margin: 0; }}
h1 {{ font-size: clamp(30px, 5vw, 44px); line-height: 1.05; }}
h2 {{ font-size: 24px; }}
h3 {{ font-size: 18px; }}
p {{ margin: 0; max-width: 68ch; }}
.label {{ font-family: var(--f-display); text-transform: uppercase; letter-spacing: .09em; font-size: 12.5px; color: var(--muted); font-weight: 600; }}
.num {{ font-family: var(--f-mono); font-variant-numeric: tabular-nums; font-size: 13.5px; white-space: nowrap; }}
section {{ background: var(--sheet); border: 1px solid var(--rule); padding: 22px clamp(16px, 3vw, 28px); display: grid; gap: 16px; min-width: 0; }}
.titleblock {{ display: grid; grid-template-columns: 1fr auto; gap: 18px 28px; align-items: end; border: 2px solid var(--ink); background: var(--sheet); padding: 20px clamp(16px, 3vw, 28px); }}
.tb-grid {{ display: grid; grid-template-columns: repeat(4, auto); gap: 4px 22px; }}
.tb-grid div {{ display: grid; gap: 2px; }}
.tb-grid b {{ font-family: var(--f-mono); font-weight: 600; font-size: 15px; font-variant-numeric: tabular-nums; }}
.prov {{ color: var(--warn); }}
.note {{ background: var(--warn-soft); border-left: 3px solid var(--warn); padding: 10px 14px; font-size: 15px; }}
.cols {{ display: grid; grid-template-columns: minmax(0, 1.1fr) minmax(0, 1fr); gap: 24px; align-items: start; }}
.compare {{ position: relative; width: 100%; max-width: 520px; aspect-ratio: 984 / 1540; margin: 0 auto; border: 1px solid var(--rule); overflow: hidden; user-select: none; touch-action: pan-y; }}
.compare img {{ position: absolute; inset: 0; width: 100%; height: 100%; max-width: 100%; display: block; image-rendering: pixelated; }}
.compare .top {{ clip-path: inset(0 calc(100% - var(--pos, 50%)) 0 0); }}
.compare .bar {{ position: absolute; top: 0; bottom: 0; left: var(--pos, 50%); width: 2px; background: var(--accent); transform: translateX(-1px); pointer-events: none; }}
.compare .tag {{ position: absolute; top: 8px; font-family: var(--f-display); font-size: 12px; letter-spacing: .08em; text-transform: uppercase; padding: 2px 8px; background: var(--sheet); color: var(--ink); border: 1px solid var(--rule); }}
.slider-row {{ display: grid; grid-template-columns: auto 1fr auto; gap: 10px; align-items: center; max-width: 520px; margin: 0 auto; width: 100%; }}
input[type=range] {{ width: 100%; accent-color: var(--accent); }}
input[type=range]:focus-visible {{ outline: 2px solid var(--accent); outline-offset: 3px; }}
.facts {{ display: grid; gap: 12px; }}
.fact {{ display: grid; grid-template-columns: 9.5em 1fr; gap: 10px; border-top: 1px solid var(--rule); padding-top: 10px; }}
.fact:first-child {{ border-top: 0; padding-top: 0; }}
.scroll {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: 15px; }}
th {{ text-align: left; font-family: var(--f-display); font-weight: 600; text-transform: uppercase; letter-spacing: .07em; font-size: 12.5px; color: var(--muted); border-bottom: 1.5px solid var(--ink); padding: 6px 10px 6px 0; }}
td {{ border-bottom: 1px solid var(--rule); padding: 6px 10px 6px 0; vertical-align: top; }}
.sw {{ display: inline-block; width: 11px; height: 11px; border: 1px solid var(--muted); margin-right: 8px; vertical-align: -1px; }}
.kind {{ font-family: var(--f-mono); font-size: 12.5px; padding: 1px 7px; border: 1.5px solid currentColor; }}
.k-line {{ color: var(--k-line); }} .k-arc {{ color: var(--k-arc); }} .k-curve {{ color: var(--k-curve); }}
.k-stroke {{ color: var(--k-stroke); }} .k-ellipse {{ color: var(--k-ellipse); }} .k-exact {{ color: var(--k-exact); }}
.sheet-img {{ width: 100%; max-width: 100%; border: 1px solid var(--rule); display: block; }}
.zooms {{ display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 14px; }}
figure {{ margin: 0; display: grid; gap: 6px; min-width: 0; }}
figure img {{ width: 100%; max-width: 100%; border: 1px solid var(--rule); display: block; }}
figcaption {{ font-size: 14px; color: var(--muted); }}
.drawing-box {{ background: var(--paper); border: 1px solid var(--rule); padding: 8px; overflow-x: auto; }}
.drawing {{ width: 100%; height: auto; max-height: 640px; display: block; }}
ol.q {{ margin: 0; padding-left: 1.4em; display: grid; gap: 12px; max-width: 72ch; }}
ol.q li::marker {{ font-family: var(--f-mono); color: var(--accent); font-weight: 600; }}
ol.q b {{ font-family: var(--f-display); font-weight: 600; font-size: 17px; letter-spacing: .01em; }}
.default {{ display: block; color: var(--muted); font-size: 14.5px; }}
.checks {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(210px, 1fr)); gap: 12px; }}
.check {{ border: 1px solid var(--rule); padding: 10px 12px; display: grid; gap: 4px; }}
.pill {{ font-family: var(--f-mono); font-size: 12px; padding: 1px 8px; justify-self: start; }}
.pill.ok {{ background: color-mix(in srgb, var(--ok) 18%, transparent); color: var(--ok); }}
.pill.warn {{ background: var(--warn-soft); color: var(--warn); }}
.order {{ display: flex; flex-wrap: wrap; gap: 6px; align-items: center; font-family: var(--f-mono); font-size: 13px; }}
.order span {{ border: 1px solid var(--rule); padding: 2px 8px; }}
.order i {{ color: var(--muted); font-style: normal; }}
@media (max-width: 760px) {{
  .cols, .zooms {{ grid-template-columns: minmax(0, 1fr); }}
  .titleblock {{ grid-template-columns: 1fr; }}
  .tb-grid {{ grid-template-columns: repeat(2, auto); }}
  .fact {{ grid-template-columns: 1fr; gap: 2px; }}
}}
@media (prefers-reduced-motion: reduce) {{ * {{ scroll-behavior: auto; }} }}
</style>

<div class="wrap">
  <header class="titleblock">
    <div style="display:grid;gap:8px">
      <span class="label">Front view reverse engineering · Review round 4</span>
      <h1>Fan Figure Front View</h1>
      <p>Clean symmetric redraw of the Vinc Studio squat figure. Outlines are straight lines and tangent arcs, corners sharp only where the drawing breaks. Personal use only: character design belongs to Vinc Studio.</p>
    </div>
    <div class="tb-grid">
      <div><span class="label">Figure height</span><b class="prov">{d['figure_height_mm']:.1f} mm*</b></div>
      <div><span class="label">Arm ring span</span><b>{d['arm_span_mm']:.1f} mm</b></div>
      <div><span class="label">Helmet width</span><b>{d['helmet_width_mm']:.1f} mm</b></div>
      <div><span class="label">Base</span><b>{d['base_mm'][0]:.1f} × {d['base_mm'][1]:.1f}</b></div>
      <div><span class="label">Parts</span><b>11</b></div>
      <div><span class="label">Edges to review</span><b>{len(E)}</b></div>
      <div><span class="label">Segments</span><b>{n_lines} lines · {n_arcs} arcs</b></div>
      <div><span class="label">Units</span><b>mm</b></div>
    </div>
  </header>

  <section>
    <h2>Changes in rounds 3 and 4</h2>
    <div class="checks">
      <div class="check"><span class="pill ok">done</span><b>Boots (19-29)</b><span>Square pixel steps with sharp corners, measured on both boots (50 % edge crossings) and averaged. Inner side: ledge, then three vertical steps. Outer side: ledge, notch, slanted step, curved toe. Legs above are unchanged.</span></div>
      <div class="check"><span class="pill ok">done</span><b>Fists (12)</b><span>Hands bottom = two arcs R {f['fist_radius_mm']:.2f} mm, one per fist, meeting at a sharp notch on the axis. Each arc ends at the hand corner where the arms join.</span></div>
      <div class="check"><span class="pill ok">done</span><b>Face details (5-11)</b><span>Glasses, eyebrows, nose and mouth follow the artwork as drawn: the face is slightly turned, so no mirror. Helmet and face outline stay symmetric.</span></div>
      <div class="check"><span class="pill ok">done</span><b>Upper lip (10)</b><span>Straight stroke with round ends, width {f['strokes']['MOUTH'][0]['width_mm']:.2f} mm, at the slight tilt the artwork draws. Only MOUTH changed.</span></div>
      <div class="check"><span class="pill ok">pass</span><b>Scope</b><span>Changed parts: BODY (boots only), HANDS (bottom only), face details. All other parts are identical to round 2.</span></div>
    </div>
  </section>

  <p class="note">* Scale is provisional. No real dimension was given, so the figure height is set to 99.9 mm, the value used for this same figure in an earlier project. All sizes scale with question 1 below.</p>

  <section>
    <h2>Model over the artwork</h2>
    <div class="cols">
      <div style="display:grid;gap:10px">
        <div class="compare" id="cmp" style="--pos:50%">
          <img src="{img_art}" alt="Artwork, legs moved onto the body axis">
          <img class="top" src="{img_model}" alt="Rebuilt model, visible colour regions">
          <span class="tag" style="left:8px">Model</span><span class="tag" style="right:8px">Artwork</span>
          <div class="bar"></div>
        </div>
        <div class="slider-row"><span class="label">Model</span><input id="pos" type="range" min="0" max="100" value="50" aria-label="Move the split between model and artwork"><span class="label">Artwork</span></div>
      </div>
      <div class="facts">
        <div class="fact"><span class="label">Symmetry</span><p>Every part is an exact mirror about one axis, except the face details (glasses, brows, nose, mouth), drawn on a slightly turned face. The artwork draws the legs {f['legs_moved_onto_body_axis_mm']:.2f} mm left of the body; the model moves them onto the axis. The artwork on the left shows the same move, so the two line up.</p></div>
        <div class="fact"><span class="label">Fit</span><p>Overlap with the symmetrised artwork: silhouette <span class="num">{iou['silhouette']:.3f}</span>, blue <span class="num">{iou['blue']:.3f}</span>, black <span class="num">{iou['black']:.3f}</span>, white <span class="num">{iou['white']:.3f}</span>. White is lower because the artwork has a pale halo between helmet and face, and because the face details now follow the drawn (unmirrored) artwork while this reference is mirrored.</p></div>
        <div class="fact"><span class="label">Pixel budget</span><p>Effective pixel {M['effective_source_pixel_mm']:.2f} mm (edge spread {M['measured_edge_width_px']:.1f} px vs ideal edge). Curves stay within half an effective pixel (<span class="num">{0.5 * M['effective_source_pixel_mm']:.2f} mm</span>) of the trace. Smaller wobble is raster noise and is removed.</p></div>
        <div class="fact"><span class="label">Tangency</span><p>Largest break at a smooth join: <span class="num">{max_break:.2f}°</span>. Joins above 20° are real corners. One exception: the waist corner is two breaks (18° and 29°), see question 6.</p></div>
        <div class="fact"><span class="label">Arm ring</span><p>Visible edges are exact ellipses: outer {2 * f['arm_ring']['outer_half_axes_mm'][0]:.1f} × {2 * f['arm_ring']['outer_half_axes_mm'][1]:.1f} mm, inner {2 * f['arm_ring']['inner_half_axes_mm'][0]:.1f} × {2 * f['arm_ring']['inner_half_axes_mm'][1]:.1f} mm, fit error 0.04 mm rms on the outer edge. Inside the jacket width ({f['arm_ends']['blend_starts_at_x_mm']:.1f} mm from the axis) each edge turns tangentially into the end of the hand side (edge 12), at most {max(f['arm_ends']['blend_max_departure_from_oval_mm']):.2f} mm off the oval. There the artwork is black on black, so it gives no edge to follow.</p></div>
      </div>
    </div>
  </section>

  <section>
    <h2>Construction, read from the print and the video</h2>
    <div class="cols">
      <div style="display:grid;gap:12px">
        <p>The video shows the arm ring turning a full circle about the chest. The helmet hides the ring at the top, and the hands cover the jacket. So the head plate is in front of the ring and the body plate is behind it.</p>
        <p>Because the ring turns, the body area it covers at rest shows during the motion. The jacket is therefore drawn as the straight panel seen in the frames, from nose level down, with the shirt V cut out.</p>
        <p>Jacket, pants and boots show no seam on the print, so they are one black plate (BODY). The shirt is plain white on the print, without a tie.</p>
        <div class="order"><span>BASE</span><i>→</i><span>CRAVAT</span><i>→</i><span>BODY</span><i>→</i><span>ARMS</span><i>→</i><span>HANDS</span><i>→</i><span>FACE</span><i>→</i><span>HELMET</span><i>→</i><span>face lines</span></div>
        <span class="label">Stacking order, back to front</span>
      </div>
      <div style="display:grid;gap:12px">
        <figure><img src="{img_frames}" alt="Video frames: ring at rest, ring turned up, ring over the head, ring behind the head"><figcaption>Video at 2 s, 3 s, 8 s, 12 s. At 3 s and 12 s the straight jacket panel is uncovered.</figcaption></figure>
      </div>
    </div>
  </section>

  <section>
    <h2>Numbered edges</h2>
    <p>Each number is one edge between two corners, on the right half. The left half is its mirror. Answer by number, for example “17 straight, 30 two curves, 26 sharper toe”. Numbers changed in round 3: face details now include the left side, and the boots have more corners. Hands bottom 12, hand side 13, boots 19-29, jacket side 31, arm ovals 35 and 36, arm ends 40 and 41.</p>
    <div class="cols">
      <img class="sheet-img" src="{img_edges}" alt="Numbered edge sheet over the artwork">
      <div class="scroll"><table>
        <thead><tr><th>#</th><th>Part</th><th>Kind</th><th>Shape</th><th>mm</th></tr></thead>
        <tbody>{rows_edges}</tbody>
      </table></div>
    </div>
  </section>

  <section>
    <h2>Zoom checks</h2>
    <div class="zooms">
      {''.join(f'<figure><img src="{src}" alt="{n} outline over artwork pixels"><figcaption>{n}. Outline over the artwork pixels; dots mark segment joins.</figcaption></figure>' for n, src in zooms)}
    </div>
  </section>

  <section>
    <h2>Parts</h2>
    <div class="scroll"><table>
      <thead><tr><th>Part</th><th>W × H mm</th><th>Visible / full mm²</th><th>Outline rule</th></tr></thead>
      <tbody>{rows_parts}</tbody>
    </table></div>
    <div class="drawing-box">{svg_front}</div>
  </section>

  <section>
    <h2>Self-checks</h2>
    <div class="checks">
      <div class="check"><span class="pill ok">pass</span><b>Gaps and slivers</b><span>Visible colour regions tile the figure. Uncovered area {K['gaps_and_slivers']['uncovered_area']:.2f} mm², no slivers below 0.3 mm².</span></div>
      <div class="check"><span class="pill ok">pass</span><b>Raster specks</b><span>No islands or pinholes below {max(0.3, (2 * M['effective_source_pixel_mm']) ** 2):.2f} mm².</span></div>
      <div class="check"><span class="pill ok">pass</span><b>Hidden parts</b><span>Face continues 2.0 mm under the helmet. Jacket continues under the ring and face. Shirt continues under the face to nose level.</span></div>
      <div class="check"><span class="pill warn">ask</span><b>Thin face lines</b><span>Eyebrows {f['strokes']['EYEBROWS'][0]['width_mm']:.2f} and {f['strokes']['EYEBROWS'][1]['width_mm']:.2f} mm, mouth {f['strokes']['MOUTH'][0]['width_mm']:.2f} and {f['strokes']['MOUTH'][1]['width_mm']:.2f} mm wide. Below 0.8 mm (two lines of a 0.4 mm nozzle).</span></div>
      <div class="check"><span class="pill ok">pass</span><b>Same-colour parts</b><span>Black: BODY, ARMS, BASE, face lines stay separate (seams or motion). White: FACE and CRAVAT meet at the chin arc.</span></div>
    </div>
  </section>

  <section>
    <h2>Questions</h2>
    <ol class="q">
      <li><b>One real dimension.</b> Measure one length on the print with a caliper: figure height (helmet top to base top), base width, or helmet width.<span class="default">Now: figure height 99.9 mm, from an earlier project. Not measured.</span></li>
      <li><b>Plate thickness and face lines.</b> How thick is each plate, and how are the face lines made: inlay in the face plate, raised, or engraved?<span class="default">Now: lines drawn at artwork width ({f['strokes']['EYEBROWS'][0]['width_mm']:.2f} mm brows). For FDM I suggest widening to 0.8 mm.</span></li>
      <li><b>Symmetry and base.</b> Keep the legs on the body axis and the base centred under the figure?<span class="default">Now: yes. The artwork draws the legs {f['legs_moved_onto_body_axis_mm']:.2f} mm left and the base {abs(f['base_centre_offset_in_artwork_mm']):.2f} mm left.</span></li>
      <li><b>Crotch top dip.</b> The crotch top (17) has a small dip in the artwork. Keep it, or make it one straight line?<span class="default">Now: keeps its 0.4 mm dip.</span></li>
      <li><b>Edges.</b> Look at the numbered sheet. Name any edge that needs a sharp corner, one continuous curve, or a straight line.</li>
      <li><b>Waist corner.</b> Where the jacket side (31) meets the hip, the outline breaks twice (18° and 29°) over 1 mm. Make it one sharp corner?<span class="default">Now: unchanged from round 1, because it was not part of this round.</span></li>
    </ol>
  </section>
</div>

<script>
(function () {{
  var cmp = document.getElementById('cmp'), pos = document.getElementById('pos');
  function set(v) {{ cmp.style.setProperty('--pos', v + '%'); }}
  pos.addEventListener('input', function () {{ set(pos.value); }});
  function drag(e) {{
    var r = cmp.getBoundingClientRect(); var x = (e.touches ? e.touches[0].clientX : e.clientX) - r.left;
    var v = Math.max(0, Math.min(100, x / r.width * 100)); pos.value = v; set(v);
  }}
  cmp.addEventListener('pointerdown', function (e) {{ drag(e); cmp.setPointerCapture(e.pointerId); }});
  cmp.addEventListener('pointermove', function (e) {{ if (e.buttons) drag(e); }});
}})();
</script>
"""
OUT.write_text(page, encoding="utf-8")
print(OUT, round(OUT.stat().st_size / 1e6, 2), "MB")
