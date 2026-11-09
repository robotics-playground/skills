"""Colour separation that removes raster defects instead of creating them.

Hard thresholds (HSV ranges etc.) leave pixel-sized bumps and coloured fringes: the 50/50 blend of
black and a light background can be nearest to a third paint (e.g. helmet blue) and forms a thin false
rim. Here every pixel is explained either as a flat paint or as a linear blend of two paints; it is
assigned to the paint it is mostly made of (alpha >= 0.5). Edges therefore land on the true 50 % line.
"""
from __future__ import annotations
import numpy as np
import cv2


def upsample(img_bgr: np.ndarray, factor: int) -> np.ndarray:
    """Cubic upsampling so contours can be traced with sub-pixel accuracy."""
    return cv2.resize(img_bgr, None, fx=factor, fy=factor, interpolation=cv2.INTER_CUBIC)


def classify(img_bgr: np.ndarray, palette: dict[str, tuple[int, int, int]], blur_px: float = 0.3,
             scale: int = 1) -> tuple[np.ndarray, list[str]]:
    """Label each pixel with a paint name index.

    img_bgr   image (already upsampled by `scale` if you want sub-pixel masks)
    palette   name -> BGR of each flat paint, INCLUDING the background
    blur_px   light blur in ORIGINAL pixels to kill compression ringing (0.3 is a good default;
              0.45 merged two strokes ~1 px apart in practice -> keep it small)
    returns   (labels HxW int array, names list)
    """
    names = list(palette)
    pal = np.array([palette[n] for n in names], np.float32)
    im = cv2.GaussianBlur(np.clip(img_bgr, 0, 255).astype(np.uint8), (0, 0), max(blur_px * scale, 0.01))
    flat = im.astype(np.float32).reshape(-1, 3)
    best = np.full(len(flat), np.inf, np.float32)
    lab = np.zeros(len(flat), np.int32)
    for i in range(len(pal)):                      # flat paints
        d = ((flat - pal[i]) ** 2).sum(1)
        m = d < best; best[m] = d[m]; lab[m] = i
    for i in range(len(pal)):                      # two-paint blends
        for j in range(i + 1, len(pal)):
            v = pal[j] - pal[i]
            a = np.clip(((flat - pal[i]) @ v) / (v @ v), 0, 1)
            d = ((flat - (pal[i] + a[:, None] * v)) ** 2).sum(1)
            m = d < best * 0.999
            best[m] = d[m]; lab[m] = np.where(a[m] < 0.5, i, j)
    return lab.reshape(im.shape[:2]), names


def sample_palette(img_bgr: np.ndarray, points: dict[str, tuple[int, int]], r: int = 1) -> dict[str, tuple[int, int, int]]:
    """Average BGR in a (2r+1)^2 window at each named (x, y) point -> palette dict."""
    out = {}
    for name, (x, y) in points.items():
        out[name] = tuple(int(v) for v in img_bgr[y - r:y + r + 1, x - r:x + r + 1].reshape(-1, 3).mean(0).round())
    return out
