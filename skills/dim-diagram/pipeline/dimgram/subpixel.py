from __future__ import annotations
import cv2
import numpy as np


def refine_contour(gray: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Largest mask contour -> simplified polygon -> sub-pixel-refined vertices."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    c = max(contours, key=cv2.contourArea).astype(np.float32)
    eps = 0.01 * cv2.arcLength(c, True)
    poly = cv2.approxPolyDP(c, eps, True).reshape(-1, 2).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    refined = poly.copy().reshape(-1, 1, 2)
    cv2.cornerSubPix(gray, refined, (5, 5), (-1, -1), criteria)
    return refined.reshape(-1, 2).astype(np.float64)
