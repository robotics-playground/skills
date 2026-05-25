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
