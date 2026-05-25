from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import cv2
import numpy as np
import yaml


@dataclass
class CameraCalib:
    camera_matrix: np.ndarray   # 3x3 K
    dist_coeffs: np.ndarray     # 1xN distortion
    reproj_error: float         # RMS reprojection error (px)
    image_size: tuple[int, int] # (w, h)


def calibrate_camera(images_bgr: list[np.ndarray], board: "cv2.aruco.CharucoBoard") -> CameraCalib:
    """Calibrate intrinsics + distortion from multiple ChArUco-board photos."""
    detector = cv2.aruco.CharucoDetector(board)
    obj_points, img_points = [], []
    size = None
    for img in images_bgr:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        cc, ci, _, _ = detector.detectBoard(gray)
        if cc is None or len(cc) < 8:
            continue
        op, ip = board.matchImagePoints(cc, ci)
        if op is None or len(op) < 8:
            continue
        obj_points.append(op)
        img_points.append(ip)
    if len(obj_points) < 3:
        raise ValueError(f"need >=3 usable views, got {len(obj_points)}")
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    return CameraCalib(K, dist, float(rms), size)


def save_camera_yaml(calib: CameraCalib, path) -> None:
    Path(path).write_text(yaml.safe_dump({
        "camera_matrix": calib.camera_matrix.tolist(),
        "dist_coeffs": np.asarray(calib.dist_coeffs).ravel().tolist(),
        "reproj_error": calib.reproj_error,
        "image_size": list(calib.image_size),
    }))


def load_camera_yaml(path) -> CameraCalib:
    d = yaml.safe_load(Path(path).read_text())
    return CameraCalib(
        camera_matrix=np.array(d["camera_matrix"], float),
        dist_coeffs=np.array(d["dist_coeffs"], float),
        reproj_error=d.get("reproj_error", float("nan")),
        image_size=tuple(d["image_size"]),
    )


def undistort_image(img: np.ndarray, calib: CameraCalib) -> np.ndarray:
    return cv2.undistort(img, calib.camera_matrix, np.asarray(calib.dist_coeffs))
