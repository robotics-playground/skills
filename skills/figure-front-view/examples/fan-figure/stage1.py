"""Colour unmixing, base/sole line, symmetry axis per body zone."""
import json
import cv2
import numpy as np
from figfront import unmix
import config as C

C.WORK.mkdir(exist_ok=True)
art = cv2.imread(str(C.ARTWORK))
H, W = art.shape[:2]
up = unmix.upsample(art, C.UP)
lab, names = unmix.classify(up, C.PALETTE, blur_px=0.3, scale=C.UP)
idx = {n: i for i, n in enumerate(names)}
fg = lab != idx["bg"]

# base = wide dark block at the bottom; sole line = first row whose fg width exceeds 85 % of the base width
rows = fg.sum(1) / C.UP
base_w = rows.max()
base_top = int(np.argmax(rows > 0.85 * base_w))
sole_y = base_top / C.UP


def mirror_axis(mask, y0, y1):
    """x (artwork px) that best maps the mask band onto its mirror image (parabolic sub-pixel peak)."""
    band = mask[int(y0 * C.UP):int(y1 * C.UP)].astype(np.float32)
    W_ = band.shape[1]; fl = band[:, ::-1]
    xs = np.nonzero(band)[1]; c = xs.mean() / C.UP
    cand = np.arange(c - 6, c + 6, 1.0 / C.UP)
    def score(x):
        M = np.roll(fl, int(round(2 * x * C.UP - (W_ - 1))), axis=1)
        return (M * band).sum() / np.maximum(M, band).sum()
    sc = np.array([score(x) for x in cand]); i = int(np.clip(sc.argmax(), 1, len(sc) - 2))
    d = sc[i - 1] - 2 * sc[i] + sc[i + 1]
    off = 0.5 * (sc[i - 1] - sc[i + 1]) / d if d < 0 else 0.0
    return float(cand[i] + off / C.UP), float(sc[i])


blue = lab == idx["blue"]
axes = {
    "head(helmet)": mirror_axis(blue, 0, 110),
    "torso+arms": mirror_axis(fg, 110, 180),
    "legs": mirror_axis(fg, 185, sole_y - 1),
    "whole": mirror_axis(fg, 0, sole_y - 1),
}
print("sole line y =", round(sole_y, 2), " base width px =", round(base_w, 1))
for k, (x, s) in axes.items(): print(f"axis {k:14s} x={x:.2f}  mirror IoU={s:.3f}")

np.savez_compressed(C.WORK / "labels.npz", lab=lab, names=np.array(names), sole_y=sole_y)
json.dump(dict(sole_y=sole_y, base_w_px=float(base_w), axes={k: [float(x), float(s)] for k, (x, s) in axes.items()}),
          open(C.WORK / "stage1.json", "w"), indent=1)

# label preview
colours = {"bg": (230, 240, 210), "blue": (200, 140, 30), "white": (255, 255, 255), "grey": (170, 170, 170), "black": (40, 40, 40)}
prev = np.zeros(up.shape, np.uint8)
for n, i in idx.items(): prev[lab == i] = colours[n]
cv2.imwrite(str(C.WORK / "labels.png"), cv2.resize(prev, None, fx=4 / C.UP, fy=4 / C.UP, interpolation=cv2.INTER_AREA))
