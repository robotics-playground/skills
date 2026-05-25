from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class RectifyResult:
    image: np.ndarray
    mm_per_px: float
    homography: np.ndarray
    tilt_deg: float
    sigma_scale: float


def _tilt_from_homography(H: np.ndarray) -> float:
    # Rough proxy: how far the metric->image map departs from affine/fronto-parallel.
    # For a square-on camera the map is a similarity and tilt ~ 0 deg.
    h1, h2 = H[:, 0], H[:, 1]
    n = np.cross(h1, h2)
    n = n / (np.linalg.norm(n) + 1e-9)
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))


def rectify_to_metric(image_bgr: np.ndarray, det, target_px_per_mm: float = 4.0) -> RectifyResult:
    src = det.charuco_corners.astype(np.float64)
    dst = det.object_points_mm * target_px_per_mm          # mm -> target px
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    tilt = _tilt_from_homography(np.linalg.inv(H))

    # Size the output canvas to the FULL transformed input image so the object
    # (which sits outside the board's extent) is not clipped. Translate so all
    # warped coordinates are positive.
    h_img, w_img = image_bgr.shape[:2]
    corners = np.array([[0, 0], [w_img, 0], [w_img, h_img], [0, h_img]],
                       np.float64).reshape(-1, 1, 2)
    warped_corners = cv2.perspectiveTransform(corners, H).reshape(-1, 2)
    min_xy = warped_corners.min(axis=0)
    max_xy = warped_corners.max(axis=0)
    T = np.array([[1, 0, -min_xy[0]], [0, 1, -min_xy[1]], [0, 0, 1]], np.float64)
    Ht = T @ H
    out_w = min(int(np.ceil(max_xy[0] - min_xy[0])), 20000)
    out_h = min(int(np.ceil(max_xy[1] - min_xy[1])), 20000)
    warped = cv2.warpPerspective(image_bgr, Ht, (out_w, out_h))

    # Fractional scale uncertainty from homography reprojection RMS over the board span.
    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    rms_px = float(np.sqrt(np.mean(np.sum((proj - dst) ** 2, axis=1))))
    span_px = float(np.linalg.norm(dst.max(axis=0) - dst.min(axis=0)))
    sigma_scale = rms_px / max(span_px, 1e-6)
    return RectifyResult(warped, 1.0 / target_px_per_mm, Ht, tilt, sigma_scale)
