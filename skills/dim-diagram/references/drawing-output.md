# Drawing Output: ezdxf 3-view dimensioned sheet

Assembles the three `ViewMeasurement`s into a CAD-style dimensioned sheet and exports
SVG / PNG / PDF / DXF. Implemented in `dimgram/draw.py`.

## Why ezdxf

`ezdxf` produces **real CAD DIMENSION entities** — auto-computed dimension text, proper
arrowheads and extension lines — and renders the same model to **SVG, PNG and PDF** through its
drawing add-on, with **no system-Cairo dependency**. The **DXF** is the native artifact, so the
user gets a CAD-editable file (FreeCAD/AutoCAD) for free.

## Building the sheet

```python
doc = ezdxf.new("R2010", setup=True)        # setup=True provides the "EZDXF" dimstyle
msp = doc.modelspace()
```

For each view, place the (mm-scaled) contour and add overall dimensions. **`.render()` is
mandatory** after every `add_*_dim()` — without it the dimension geometry is never built:

```python
msp.add_lwpolyline(points_mm, close=True)
msp.add_linear_dim(base=..., p1=..., p2=..., angle=0, dimstyle="EZDXF",
                   override={"dimtxt": 5.0, "dimasz": 3.0, "dimexe": 2.0,
                             "dimexo": 2.0, "dimdec": 1}).render()
```

`override` controls legibility at object scale: `dimtxt` text height, `dimasz` arrow size,
`dimexe`/`dimexo` extension-line overshoot/offset, `dimdec` decimal places (all mm).

## Export to all formats

One renderer, all formats — via the matplotlib backend of the drawing add-on:

```python
from ezdxf.addons.drawing import matplotlib as ezmpl
doc.saveas(path.with_suffix(".dxf"))                      # native DXF
for ext in ("png", "pdf", "svg"):
    ezmpl.qsave(msp, str(path.with_suffix("." + ext)))    # raster + vector
```

(Alternative backends exist: `SVGBackend` for pure SVG, `PyMuPdfBackend` for PDF.)

## Layout

`LAYOUT` places the three views in **third-angle** arrangement (top above front, side to the
right), in millimetre modelspace coordinates, with the title block below. Third-angle vs
first-angle: in third-angle the top view sits *above* the front and the right-side view to its
*right* (US/ASME default). Keep views in projection alignment and stamp the convention in the
title block (`"... | units: mm | 3rd-angle"`).

## Fallback renderer (planned)

A pure-matplotlib renderer (`ax.annotate(arrowstyle="<->")` / `FancyArrowPatch`, one Figure →
SVG+PDF+PNG via `savefig`) is a planned alternative for environments without ezdxf. Keep figure
dpi equal to savefig dpi to avoid arrowhead-scaling drift.

## References
- ezdxf drawing add-on: https://ezdxf.readthedocs.io/en/stable/addons/drawing.html
- ezdxf linear dimensions: https://ezdxf.readthedocs.io/en/stable/tutorials/linear_dimension.html
- First vs third angle: https://www.gdandtbasics.com/first-vs-third-angle-orthographic-views/
