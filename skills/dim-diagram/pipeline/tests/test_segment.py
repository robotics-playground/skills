import cv2
import numpy as np
from dimgram.segment import ClassicalSegmenter


def test_segment_finds_rectangle():
    img = np.full((200, 300, 3), 255, np.uint8)
    cv2.rectangle(img, (50, 40), (250, 160), (0, 0, 0), -1)   # 200 x 120 px black box
    mask = ClassicalSegmenter().segment(img)
    assert mask.dtype == np.uint8
    ys, xs = np.where(mask > 0)
    assert abs((xs.max() - xs.min()) - 200) <= 3
    assert abs((ys.max() - ys.min()) - 120) <= 3
