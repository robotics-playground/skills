"""Review images: outlines over the artwork pixels, assembled colour view, zoom crops."""
import pickle
import sys
import cv2
import numpy as np
from shapely.ops import unary_union
from figfront import layers
import config as C

D = pickle.load(open(C.WORK / "parts_px.pkl", "rb"))
parts = D["parts"]
art = cv2.imread(str(C.ARTWORK))
PAINT = {"white": (250, 250, 250), "black": (40, 40, 40), "blue": (210, 150, 40)}
LINE = {"CRAVAT": (0, 160, 255), "BODY": (0, 0, 255), "ARMS": (0, 200, 0), "HANDS": (255, 0, 255), "FACE": (255, 120, 0),
        "HELMET": (0, 0, 160), "GLASSES": (0, 255, 255), "EYEBROWS": (255, 255, 0), "NOSE": (120, 0, 255),
        "MOUTH": (0, 255, 120), "BASE": (128, 0, 128)}


def draw(img, g, col, S, x0=0, y0=0, th=1):
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        for r in [p.exterior, *p.interiors]:
            P = ((np.array(r.coords) - [x0, y0]) * S).round().astype(np.int32)
            cv2.polylines(img, [P], True, col, th, cv2.LINE_AA)


def fill(img, g, col, S, x0=0, y0=0):
    for p in getattr(g, "geoms", [g]):
        if p.is_empty: continue
        m = np.zeros(img.shape[:2], np.uint8)
        cv2.fillPoly(m, [((np.array(p.exterior.coords) - [x0, y0]) * S).round().astype(np.int32)], 1)
        for h in p.interiors: cv2.fillPoly(m, [((np.array(h.coords) - [x0, y0]) * S).round().astype(np.int32)], 0)
        img[m.astype(bool)] = col


import json
FITS = json.load(open(C.WORK / "fits.json"))


def art_corrected(S):
    """Artwork upscaled (nearest) with the legs moved onto the body axis, as the model does."""
    big = cv2.resize(art, None, fx=S, fy=S, interpolation=cv2.INTER_NEAREST)
    d = int(round((FITS["AX"] - FITS["AX_LEGS"]) * S)); y = int(round(FITS["Y_SPLIT"] * S))
    yb = int(round(FITS["sole_y"] * S))
    big[y:yb] = np.roll(big[y:yb], d, axis=1)
    return big


def overlay(S=4, crop=None, keys=None, name="overlay.png", th=1):
    x0, y0, x1, y1 = crop or (0, 0, art.shape[1], art.shape[0])
    img = art_corrected(S)[y0 * S:y1 * S, x0 * S:x1 * S].copy()
    img = (img * 0.6 + 90).astype(np.uint8)
    for k in keys or parts:
        draw(img, parts[k], LINE.get(k, (0, 0, 255)), S, x0, y0, th)
        for so, hs in D.get("segs", {}).get(k, []):                     # segment joins as dots
            for sg in so + [x for h in hs for x in h]:
                p = ((sg.a - [x0, y0]) * S).round().astype(int)
                cv2.circle(img, tuple(int(v) for v in p), 2, (0, 0, 0), -1)
    cv2.imwrite(str(C.WORK / name), img)
    return img


def assembled(S=4, name="assembled.png"):
    img = np.full((art.shape[0] * S, art.shape[1] * S, 3), (235, 245, 225), np.uint8)
    vis = layers.visible_regions(parts, ["BASE"] + C.ORDER)
    for k in ["BASE"] + C.ORDER:
        fill(img, vis[k], PAINT[C.COLOUR[k]], S)
    for k in ["BASE"] + C.ORDER:
        draw(img, vis[k], (140, 140, 140), S)
    cv2.imwrite(str(C.WORK / name), img)
    return img


if __name__ == "__main__":
    a = overlay(4)
    b = assembled(4)
    ref = art_corrected(4)
    cv2.imwrite(str(C.WORK / "side_by_side.png"), np.hstack([ref, b]))
    overlay(10, (60, 0, 200, 118), name="z_head.png", th=1)
    overlay(10, (70, 95, 190, 175), name="z_chest.png", th=1)
    overlay(8, (50, 170, 210, 305), name="z_legs.png", th=1)
