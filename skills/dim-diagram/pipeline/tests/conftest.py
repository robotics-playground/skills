import cv2
import numpy as np
import pytest

# Board geometry shared across tests (matches the A4 default in board.py).
DICT = cv2.aruco.DICT_5X5_1000
SQUARES_X, SQUARES_Y = 7, 10
SQUARE_MM, MARKER_MM = 24.0, 18.0


@pytest.fixture
def board():
    d = cv2.aruco.getPredefinedDictionary(DICT)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_MM, MARKER_MM, d)


@pytest.fixture
def synthetic_scene(board):
    """A fronto-parallel scene at a known scale: board on the left, a known
    100 x 60 mm rectangle on the right. Returns (image, mm_per_px, rect_mm)."""
    mm_per_px = 0.25                      # 4 px per mm
    px = lambda mm: int(round(mm / mm_per_px))
    board_w_mm, board_h_mm = SQUARES_X * SQUARE_MM, SQUARES_Y * SQUARE_MM
    margin = px(20)
    rect_mm = (100.0, 60.0)
    canvas_h = margin * 2 + px(board_h_mm)
    canvas_w = margin * 3 + px(board_w_mm) + px(rect_mm[0])
    img = np.full((canvas_h, canvas_w), 255, np.uint8)

    board_img = board.generateImage((px(board_w_mm), px(board_h_mm)))
    img[margin:margin + px(board_h_mm), margin:margin + px(board_w_mm)] = board_img

    rx0 = margin * 2 + px(board_w_mm)
    ry0 = margin
    cv2.rectangle(img, (rx0, ry0), (rx0 + px(rect_mm[0]), ry0 + px(rect_mm[1])), 0, -1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), mm_per_px, rect_mm
