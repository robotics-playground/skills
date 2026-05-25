from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class CharucoDetection:
    charuco_corners: np.ndarray   # (N, 2) sub-pixel image coords
    charuco_ids: np.ndarray       # (N, 1)
    object_points_mm: np.ndarray  # (N, 2) corresponding board coords in mm


def detect_charuco(image_bgr: np.ndarray, board: "cv2.aruco.CharucoBoard"):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    ch_params = cv2.aruco.CharucoParameters()
    detector = cv2.aruco.CharucoDetector(board, ch_params, params)
    ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    if ch_corners is None or len(ch_corners) < 8:
        return None
    ch_corners = ch_corners.reshape(-1, 2)
    ch_ids = ch_ids.reshape(-1, 1)
    # board interior-corner coordinates in mm (z=0 plane)
    all_obj = board.getChessboardCorners()[:, :2]   # (K, 2) in mm
    obj = all_obj[ch_ids.flatten()]
    return CharucoDetection(ch_corners.astype(np.float64), ch_ids, obj.astype(np.float64))
