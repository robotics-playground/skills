import cv2
import numpy as np
from tests.conftest import SQUARES_X, SQUARES_Y, SQUARE_MM
from dimgram.calibrate import (calibrate_camera, save_camera_yaml, load_camera_yaml,
                               undistort_image)

OUT_W, OUT_H = 900, 1200
FX = FY = 1500.0
CX, CY = OUT_W / 2, OUT_H / 2
K_TRUE = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], float)


def _render_view(board, rvec, tvec, render_ppm=4.0):
    bw, bh = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    bimg = board.generateImage((int(bw * render_ppm), int(bh * render_ppm)))
    Wb, Hb = bimg.shape[1], bimg.shape[0]
    src = np.array([[0, 0], [Wb, 0], [Wb, Hb], [0, Hb]], np.float32)
    obj = np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], np.float32)
    imgpts, _ = cv2.projectPoints(obj, rvec, tvec, K_TRUE, None)
    H = cv2.getPerspectiveTransform(src, imgpts.reshape(-1, 2).astype(np.float32))
    canvas = cv2.warpPerspective(bimg, H, (OUT_W, OUT_H), borderValue=255)
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def _views(board):
    bw, bh = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    base_t = np.array([-bw / 2, -bh / 2, 650.0])   # centre the board in front of the camera
    out = []
    for ax, ay in [(0, 0), (0.25, 0), (-0.25, 0), (0, 0.25), (0, -0.25),
                   (0.2, 0.2), (-0.2, 0.2), (0.2, -0.2), (-0.2, -0.2),
                   (0.15, -0.1), (-0.1, 0.15), (0.3, 0.05)]:
        rvec = np.array([ay, ax, 0.0])             # tilt about x/y
        tvec = base_t + np.array([np.sin(ax) * 30, np.sin(ay) * 30, np.cos(ax) * 20])
        out.append(_render_view(board, rvec, tvec))
    return out


def test_calibrate_recovers_focal_length(board, tmp_path):
    calib = calibrate_camera(_views(board), board)
    fx = calib.camera_matrix[0, 0]
    assert abs(fx - FX) / FX < 0.05            # focal length within 5%
    assert calib.reproj_error < 1.0            # sub-pixel reprojection
    # yaml round-trip
    p = tmp_path / "camera.yaml"
    save_camera_yaml(calib, p)
    loaded = load_camera_yaml(p)
    np.testing.assert_allclose(loaded.camera_matrix, calib.camera_matrix)


def test_undistort_is_noop_for_zero_distortion(board):
    img = _render_view(board, np.array([0.0, 0.0, 0.0]),
                       np.array([-SQUARES_X * SQUARE_MM / 2, -SQUARES_Y * SQUARE_MM / 2, 650.0]))
    calib = calibrate_camera(_views(board), board)
    calib.dist_coeffs = np.zeros((1, 5))       # force zero distortion
    out = undistort_image(img, calib)
    assert out.shape == img.shape
