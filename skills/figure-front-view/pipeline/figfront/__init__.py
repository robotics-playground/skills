"""figfront: reusable building blocks for reverse-engineering the front view of a flat (2.5D) figure/toy
from straight-on artwork + photos of the real print.

Modules
-------
unmix      colour separation by anti-alias unmixing (pixel = flat paint or blend of two paints)
calibrate  measure the image's real resolution (edge spread vs. an ideal edge) and derive tolerances
contours   sub-pixel contours, round (disk) morphology, bounded smoothing with real-corner rebuild
geom       exact shapes: circles, oval rings, straight-line V, flat edge with bulge, face/head shape,
           sharp corners from tangents, continuous spline edges
layers     stacking order -> gap-free visible regions; hidden-part completion helpers
export     DXF (true splines split at corners, one layer per part) and SVG output
regularize simplest faithful curve per edge (line < arc < spline), corners kept (Hoshyari 2018 / ShipShape)
checks     automatic self-review: scope guard, overlap under front parts, gaps/slivers, thin features,
           same-colour neighbours, and the self-review question list
review     one consolidated numbered review sheet + legend for the user
"""
__all__ = ["unmix", "calibrate", "contours", "geom", "layers", "export", "regularize", "checks", "review"]
