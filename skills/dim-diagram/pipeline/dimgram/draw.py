from __future__ import annotations
from pathlib import Path
import ezdxf
from ezdxf.addons.drawing import matplotlib as ezmpl
from .types import ViewMeasurement

# third-angle row layout offsets (mm) in the drawing's modelspace
LAYOUT = {"front": (0.0, 0.0), "top": (0.0, 120.0), "side": (160.0, 0.0)}


def _add_view(msp, vm: ViewMeasurement, origin: tuple[float, float]) -> None:
    ox, oy = origin
    pts = [(ox + x * vm.mm_per_px, oy + y * vm.mm_per_px) for x, y in vm.contour]
    msp.add_lwpolyline(pts, close=True)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    # one horizontal + one vertical overall dimension from the contour bbox.
    # dimtxt/dimasz set text + arrow size (mm) for legibility at object scale.
    dimattr = {"dimtxt": 5.0, "dimasz": 3.0, "dimexe": 2.0, "dimexo": 2.0, "dimdec": 1}
    msp.add_linear_dim(base=(ox, max(ys) + 14), p1=(min(xs), max(ys)),
                       p2=(max(xs), max(ys)), angle=0, dimstyle="EZDXF",
                       override=dimattr).render()
    msp.add_linear_dim(base=(min(xs) - 14, oy), p1=(min(xs), min(ys)),
                       p2=(min(xs), max(ys)), angle=90, dimstyle="EZDXF",
                       override=dimattr).render()
    msp.add_text(vm.view.upper(), dxfattribs={"height": 4}).set_placement((ox, min(ys) - 10))


def render_sheet(views: dict[str, ViewMeasurement], out_stem, title: str = "Part",
                 formats=("svg", "png", "pdf", "dxf")) -> dict:
    doc = ezdxf.new("R2010", setup=True)
    msp = doc.modelspace()
    for name, vm in views.items():
        _add_view(msp, vm, LAYOUT[name])
    msp.add_text(f"{title}  |  units: mm  |  3rd-angle",
                 dxfattribs={"height": 5}).set_placement((0, -30))

    out_stem = Path(out_stem)
    out: dict[str, str] = {}
    if "dxf" in formats:
        p = out_stem.with_suffix(".dxf")
        doc.saveas(p)
        out["dxf"] = str(p)
    for ext in ("png", "pdf", "svg"):
        if ext in formats:
            p = out_stem.with_suffix("." + ext)
            ezmpl.qsave(msp, str(p))
            out[ext] = str(p)
    return out
