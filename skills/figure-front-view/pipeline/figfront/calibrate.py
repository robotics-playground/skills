"""Measure how much real detail the image holds, and derive the raster-defect tolerances from it.

Never hand-pick a smoothing tolerance. In the first project a fixed 1.2 px limit erased real drawn
features (the notch between two touching fists, thumb-base corners); 0.45 px was still a guess.
Measure instead:
  * nominal pixel width  = mm per image pixel (from a known dimension)
  * edge spread          = 10-90 % width of real edges, compared with a SIMULATED ideal anti-aliased
                           edge sampled the same way (the sampler itself adds blur)
  * effective pixel      = the source resolution that explains the measured spread
  * edge noise           = scatter of the sub-pixel edge around a local quadratic fit
Derived budget: max edge shift = 1/4 effective pixel (sub-pixel precision limit),
smoothing sigma = max(noise, 1/8 effective px), corner scale = 1 effective pixel.
"""
from __future__ import annotations
import numpy as np
import cv2
from scipy.ndimage import map_coordinates


def edge_widths(img_bgr: np.ndarray, ring: np.ndarray, paint_a, paint_b, step: float = 1.5,
                half: float = 4.0, ns: int = 81):
    """10-90 % widths (px) of the colour transition A|B sampled along normals of `ring` (Nx2, x,y)."""
    im = img_bgr.astype(np.float32); A = np.float32(paint_a); B = np.float32(paint_b); v = A - B
    seg = np.diff(np.vstack([ring, ring[:1]]), axis=0); L = np.linalg.norm(seg, axis=1)
    cum = np.r_[0, np.cumsum(L)]; widths = []
    for d in np.arange(0, cum[-1], step):
        i = min(np.searchsorted(cum, d, side='right') - 1, len(ring) - 1)
        t = seg[i] / (L[i] + 1e-12); p = ring[i] + t * (d - cum[i]); n = np.array([-t[1], t[0]])
        ts = np.linspace(-half, half, ns); pts = p[None] + ts[:, None] * n[None]
        col = np.stack([map_coordinates(im[:, :, c], [pts[:, 1], pts[:, 0]], order=1) for c in range(3)], 1)
        a = np.clip(((col - B) @ v) / (v @ v), -.2, 1.2)
        if a[0] > a[-1]: a = a[::-1]
        if a[:10].mean() > .15 or a[-10:].mean() < .85: continue        # not an A|B edge here
        widths.append(ts[np.argmax(a >= .9)] - ts[np.argmax(a >= .1)])
    return np.array(widths)


def simulated_widths(source_px=(1.0, 1.5, 2.0, 2.5, 3.0), trials: int = 200, seed: int = 0) -> dict:
    """10-90 % width of an ideal box-filtered step edge, rendered at coarser source pixels and resampled
    to the image grid, measured with the same bilinear normal sampler as `edge_widths`."""
    rng = np.random.default_rng(seed); out = {}
    for sc in source_px:
        ws = []
        for _ in range(trials):
            off = rng.uniform(0, 1); ang = rng.uniform(0, np.pi / 2); N = 64; ss = 8
            nx, ny = np.cos(ang), np.sin(ang); g = (np.arange(N * ss) + .5) / ss
            fine = ((g[None, :] * nx + g[:, None] * ny) > (N / 2 * (nx + ny) + off)).astype(float)
            if sc > 1:
                k = int(round(sc * ss)); M = (N * ss) // k
                coarse = fine[:M * k, :M * k].reshape(M, k, M, k).mean((1, 3))
                img = cv2.resize(coarse.astype(np.float32), (N, N), interpolation=cv2.INTER_CUBIC)
            else:
                img = fine.reshape(N, ss, N, ss).mean((1, 3))
            base = np.array([(N / 2 * (nx + ny) + off) * nx, (N / 2 * (nx + ny) + off) * ny])
            t = np.linspace(-4, 4, 81); pts = base[None] + t[:, None] * np.array([nx, ny])[None]
            a = map_coordinates(img, [pts[:, 1], pts[:, 0]], order=1)
            ws.append(t[np.argmax(a >= .9)] - t[np.argmax(a >= .1)])
        out[float(sc)] = float(np.median(ws))
    return out


def edge_noise(ring: np.ndarray, window_px: float, step: float = 0.25) -> float:
    """RMS scatter of a sub-pixel contour around local quadratic fits (+-window_px)."""
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ring, axis=0), axis=1))]; t = np.arange(0, d[-1], step)
    P = np.c_[np.interp(t, d, ring[:, 0]), np.interp(t, d, ring[:, 1])]; h = max(2, int(window_px / step)); res = []
    for i in range(h, len(P) - h, 4):
        seg = P[i - h:i + h + 1]; tt = np.linspace(-1, 1, len(seg))
        fx = np.polyval(np.polyfit(tt, seg[:, 0], 2), tt); fy = np.polyval(np.polyfit(tt, seg[:, 1], 2), tt)
        tan = np.gradient(np.c_[fx, fy], axis=0)[h]; tan /= np.linalg.norm(tan) + 1e-12
        res.append((seg[h] - [fx[h], fy[h]]) @ np.array([-tan[1], tan[0]]))
    return float(np.sqrt(np.mean(np.square(res)))) if res else 0.0


def metrics(mm_per_px: float, measured_width_px: float, noise_px: float, sim: dict | None = None) -> dict:
    """Derive the raster-defect budget from measurements (all lengths in image px unless *_mm)."""
    sim = sim or simulated_widths()
    sc = np.array(list(sim)); wv = np.array(list(sim.values()))
    eff = float(np.interp(measured_width_px, wv, sc))
    m = dict(nominal_mm_per_px=mm_per_px, measured_edge_width_px=measured_width_px, simulated_widths=sim,
             effective_source_pixel_px=round(eff, 2), effective_source_pixel_mm=round(eff * mm_per_px, 3),
             edge_noise_rms_px=round(noise_px, 3))
    m['max_shift_px'] = round(0.25 * m['effective_source_pixel_px'], 4)
    m['smoothing_sigma_px'] = round(max(noise_px, eff / 8), 3)
    m['corner_scale_px'] = round(eff, 2)
    m['max_shift_mm'] = round(m['max_shift_px'] * mm_per_px, 3)
    return m
