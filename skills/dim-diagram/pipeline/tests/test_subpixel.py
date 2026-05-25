import cv2
import numpy as np
from dimgram.subpixel import refine_contour


def test_refine_contour_returns_subpixel_polygon():
    img = np.full((200, 300, 3), 255, np.uint8)
    cv2.rectangle(img, (50, 40), (250, 160), (0, 0, 0), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(gray.shape, np.uint8)
    cv2.rectangle(mask, (50, 40), (250, 160), 255, -1)
    poly = refine_contour(gray, mask)
    assert poly.shape[1] == 2
    assert poly.dtype == np.float64
    # corners should sit near the true rectangle corners
    assert np.any(np.linalg.norm(poly - np.array([50, 40]), axis=1) < 2.0)
