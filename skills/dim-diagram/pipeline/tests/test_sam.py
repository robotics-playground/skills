"""SamSegmenter test. Skipped automatically if ultralytics/torch or the MobileSAM
weights aren't available (the optional [sam] extra)."""
import cv2
import numpy as np
import pytest

pytest.importorskip("ultralytics")


def _scene_on_textured_bg(mm_per_px=0.25):
    px = lambda mm: int(round(mm / mm_per_px))
    rng = np.random.default_rng(0)
    img = rng.integers(120, 200, (px(90) + 80, px(120) + 80, 3), dtype=np.uint8)  # noisy bg
    cv2.rectangle(img, (40, 40), (40 + px(120), 40 + px(90)), (20, 20, 20), -1)
    return img, (px(120), px(90))


def test_sam_segments_object():
    from dimgram.segment import SamSegmenter
    img, (w_px, h_px) = _scene_on_textured_bg()
    try:
        seg = SamSegmenter()
    except Exception as e:                      # weights download / offline
        pytest.skip(f"MobileSAM unavailable: {e}")
    mask = seg.segment(img)
    assert mask.shape == img.shape[:2]
    ys, xs = np.where(mask > 0)
    assert xs.size > 0
    # recovered bbox should roughly match the rectangle (SAM is coarse -> loose tolerance)
    assert abs((xs.max() - xs.min()) - w_px) < 0.15 * w_px
    assert abs((ys.max() - ys.min()) - h_px) < 0.15 * h_px
