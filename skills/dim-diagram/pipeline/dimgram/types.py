from __future__ import annotations
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Optional
import numpy as np

Axis = Literal["X", "Y", "Z"]
View = Literal["front", "top", "side"]


@dataclass
class Dimension:
    label: str
    kind: Literal["linear", "diameter", "radius"]
    p1: tuple[float, float]
    p2: tuple[float, float]
    value_mm: float
    sigma_mm: float
    axis: Optional[Axis] = None


@dataclass
class ViewMeasurement:
    view: View
    dimensions: list[Dimension]
    contour: np.ndarray            # (N, 2) float, rectified-image px (sub-pixel)
    mm_per_px: float
    warnings: list[str] = field(default_factory=list)


def to_json(vm: ViewMeasurement, path: Path) -> None:
    # Coerce to native Python floats so numpy scalars (e.g. float32 from cv2) serialize.
    payload = {
        "view": vm.view,
        "dimensions": [{
            "label": d.label,
            "kind": d.kind,
            "p1": [float(d.p1[0]), float(d.p1[1])],
            "p2": [float(d.p2[0]), float(d.p2[1])],
            "value_mm": float(d.value_mm),
            "sigma_mm": float(d.sigma_mm),
            "axis": d.axis,
        } for d in vm.dimensions],
        "contour": np.asarray(vm.contour, dtype=float).tolist(),
        "mm_per_px": float(vm.mm_per_px),
        "warnings": list(vm.warnings),
    }
    Path(path).write_text(json.dumps(payload, indent=2))


def from_json(path: Path) -> ViewMeasurement:
    payload = json.loads(Path(path).read_text())
    dims = [Dimension(**{**d, "p1": tuple(d["p1"]), "p2": tuple(d["p2"])})
            for d in payload["dimensions"]]
    return ViewMeasurement(
        view=payload["view"],
        dimensions=dims,
        contour=np.array(payload["contour"], dtype=float),
        mm_per_px=payload["mm_per_px"],
        warnings=payload.get("warnings", []),
    )
