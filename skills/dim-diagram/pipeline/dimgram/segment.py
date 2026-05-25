from __future__ import annotations
from typing import Protocol
import cv2
import numpy as np


class Segmenter(Protocol):
    def segment(self, image_bgr: np.ndarray) -> np.ndarray: ...


class ClassicalSegmenter:
    """Otsu threshold + largest-contour fill. Assumes a contrasting background."""

    def segment(self, image_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return np.zeros(gray.shape, np.uint8)
        biggest = max(contours, key=cv2.contourArea)
        mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(mask, [biggest], -1, 255, -1)
        return mask


class SamSegmenter:
    """MobileSAM (via the optional `ultralytics` extra) for cluttered backgrounds.

    Localises the object from a foreground point prompt (image centre by default).
    Robust to non-uniform backgrounds where ClassicalSegmenter fails. Note: the SAM
    mask is pixel-coarse — measurement still relies on the sub-pixel refinement stage.
    Install with: pip install '.[sam]'."""

    def __init__(self, model: str = "mobile_sam.pt", point: tuple[int, int] | None = None):
        from ultralytics import SAM   # lazy: keeps torch out of the core install
        self._model = SAM(model)
        self._point = point

    def segment(self, image_bgr: np.ndarray) -> np.ndarray:
        h, w = image_bgr.shape[:2]
        px, py = self._point or (w // 2, h // 2)
        res = self._model.predict(image_bgr, points=[[px, py]], labels=[1], verbose=False)
        masks = res[0].masks
        if masks is None or len(masks.data) == 0:
            return np.zeros((h, w), np.uint8)
        m = masks.data[0].cpu().numpy()
        mask = (m > 0.5).astype(np.uint8) * 255
        if mask.shape != (h, w):
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        return mask
