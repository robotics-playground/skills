from __future__ import annotations
import numpy as np
from .types import ViewMeasurement

DISAGREE_PCT = 0.015   # flag shared-axis disagreement above 1.5%


def reconcile(views: dict[str, ViewMeasurement], tol_pct: float = DISAGREE_PCT) -> dict:
    by_axis: dict[str, list[tuple[float, float]]] = {"X": [], "Y": [], "Z": []}
    for vm in views.values():
        for d in vm.dimensions:
            if d.axis in by_axis:
                by_axis[d.axis].append((d.value_mm, max(d.sigma_mm, 1e-6)))

    axes_mm, warnings = {}, []
    for axis, vals in by_axis.items():
        if not vals:
            continue
        v = np.array([x[0] for x in vals])
        w = 1.0 / np.array([x[1] for x in vals]) ** 2
        mean = float(np.sum(v * w) / np.sum(w))
        axes_mm[axis] = round(mean, 2)
        if len(v) > 1 and (v.max() - v.min()) / mean > tol_pct:
            warnings.append(f"axis {axis} disagreement: {v.tolist()} mm "
                            f"({100 * (v.max() - v.min()) / mean:.1f}%)")
    return {"axes_mm": axes_mm, "warnings": warnings, "views": views}
