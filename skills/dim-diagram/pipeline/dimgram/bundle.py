from __future__ import annotations
from pathlib import Path
import cv2
import numpy as np
import yaml
from .types import ViewMeasurement, to_json


def write_bundle(bundle_dir, views: dict[str, ViewMeasurement],
                 rectified: dict[str, np.ndarray], mm_per_px: float,
                 board_yaml: dict, camera_yaml: dict) -> Path:
    bundle_dir = Path(bundle_dir)
    (bundle_dir / "rectified").mkdir(parents=True, exist_ok=True)
    for name, vm in views.items():
        to_json(vm, bundle_dir / f"{name}.json")
        cv2.imwrite(str(bundle_dir / "rectified" / f"{name}.png"), rectified[name])
    (bundle_dir / "scale.yaml").write_text(
        yaml.safe_dump({"mm_per_px": mm_per_px, "board": board_yaml}))
    (bundle_dir / "camera.yaml").write_text(yaml.safe_dump(camera_yaml))
    return bundle_dir
