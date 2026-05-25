import numpy as np
from dimgram.reference import detect_charuco
from dimgram.rectify import rectify_to_metric


def test_rectify_recovers_scale(synthetic_scene, board):
    img, _, _ = synthetic_scene
    det = detect_charuco(img, board)
    res = rectify_to_metric(img, det, target_px_per_mm=4.0)
    assert abs(res.mm_per_px - 0.25) < 0.01          # 4 px per mm
    assert res.image.shape[0] > 0 and res.image.shape[1] > 0
    assert res.tilt_deg < 1.0                         # synthetic scene is fronto-parallel
