from dimgram.reference import detect_charuco


def test_detect_charuco_on_synthetic(synthetic_scene, board):
    img, mm_per_px, _ = synthetic_scene
    det = detect_charuco(img, board)
    assert det is not None
    assert det.charuco_corners.shape[0] >= 12      # plenty of interior corners
    assert det.charuco_corners.shape[1] == 2
    assert det.object_points_mm.shape[0] == det.charuco_corners.shape[0]
