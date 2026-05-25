from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml
from . import board as board_mod
from .calibrate import calibrate_camera, save_camera_yaml, load_camera_yaml, undistort_image
from .reference import detect_charuco
from .rectify import rectify_to_metric
from .segment import ClassicalSegmenter
from .measure import measure_view
from .reconcile import reconcile
from .draw import render_sheet
from .bundle import write_bundle
from .types import to_json, from_json


def _load_board(asset_dir: Path):
    cfg = yaml.safe_load((Path(asset_dir) / "board.yaml").read_text())
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["dictionary"]))
    board = cv2.aruco.CharucoBoard((cfg["squares_x"], cfg["squares_y"]),
                                   cfg["square_mm"], cfg["marker_mm"], d)
    return board, cfg


def _board_mask_box(det, res, square_mm: float):
    """Bounding box (rectified px) covering the board, padded by ~1.3 squares."""
    pad = 1.3 * square_mm / res.mm_per_px
    bc = cv2.perspectiveTransform(
        det.charuco_corners.reshape(-1, 1, 2).astype(np.float64),
        res.homography).reshape(-1, 2)
    return (bc[:, 0].min() - pad, bc[:, 1].min() - pad,
            bc[:, 0].max() + pad, bc[:, 1].max() + pad)


def _rect_path(json_path: str) -> Path:
    p = Path(json_path)
    return p.parent / (p.stem + ".rect.png")


def cmd_make_board(a):
    out = board_mod.make_board(a.out, preset=a.paper, square_mm=a.square_mm)
    print(f"wrote board to {a.out}: {out['squares_x']}x{out['squares_y']} squares, "
          f"{out['square_mm']}mm each")


def cmd_calibrate(a):
    import glob
    board, _ = _load_board(a.assets)
    paths = sorted(p for ext in ("jpg", "jpeg", "png", "JPG", "JPEG", "PNG")
                   for p in glob.glob(str(Path(a.images) / f"*.{ext}")))
    imgs = [cv2.imread(p) for p in paths]
    imgs = [im for im in imgs if im is not None]
    if len(imgs) < 3:
        raise SystemExit(f"need >=3 readable images in {a.images}, found {len(imgs)}")
    calib = calibrate_camera(imgs, board)
    save_camera_yaml(calib, a.out)
    print(f"calibrated from {len(imgs)} images: reproj={calib.reproj_error:.3f}px -> {a.out}")


def cmd_process(a):
    board, cfg = _load_board(a.assets)
    img = cv2.imread(a.image)
    if img is None:
        raise SystemExit(f"cannot read image: {a.image}")
    if a.camera:
        img = undistort_image(img, load_camera_yaml(a.camera))   # undistort before detection
    det = detect_charuco(img, board)
    if det is None:
        raise SystemExit(f"no ChArUco board detected in {a.image}")
    res = rectify_to_metric(img, det, target_px_per_mm=a.px_per_mm)
    mask_box = _board_mask_box(det, res, cfg["square_mm"])
    if a.segmenter == "sam":
        from .segment import SamSegmenter
        segmenter = SamSegmenter()
    else:
        segmenter = ClassicalSegmenter()
    vm = measure_view(res, segmenter, a.view, board_mask_px=mask_box)
    to_json(vm, a.out)
    cv2.imwrite(str(_rect_path(a.out)), res.image)   # for the downstream bundle
    print(f"{a.view}: " + ", ".join(
        f"{d.label}={d.value_mm}+/-{d.sigma_mm}mm" for d in vm.dimensions)
        + (f"  [tilt {res.tilt_deg:.1f} deg]" if res.tilt_deg >= 2.5 else ""))


def cmd_assemble(a):
    paths = {"front": a.front, "top": a.top, "side": a.side}
    views = {name: from_json(p) for name, p in paths.items()}
    rec = reconcile(views)
    out = render_sheet(views, a.out, title=a.title)
    if a.bundle:
        rectified = {}
        for name, p in paths.items():
            rp = _rect_path(p)
            rectified[name] = (cv2.imread(str(rp)) if rp.exists()
                               else np.zeros((4, 4, 3), np.uint8))
        write_bundle(a.bundle, views, rectified,
                     mm_per_px=next(iter(views.values())).mm_per_px,
                     board_yaml={}, camera_yaml={"model": "none"})
    print("reconciled axes (mm):", rec["axes_mm"])
    for w in rec["warnings"]:
        print("WARNING:", w)
    print("wrote:", out)


def main(argv=None):
    p = argparse.ArgumentParser(prog="dimgram")
    sub = p.add_subparsers(required=True)

    mb = sub.add_parser("make-board")
    mb.set_defaults(fn=cmd_make_board)
    mb.add_argument("out")
    mb.add_argument("--paper", default="A4")
    mb.add_argument("--square-mm", type=float, default=None, dest="square_mm")

    cal = sub.add_parser("calibrate")
    cal.set_defaults(fn=cmd_calibrate)
    cal.add_argument("images", help="directory of ChArUco-board photos")
    cal.add_argument("--out", required=True, help="output camera.yaml")
    cal.add_argument("--assets",
                     default=str(Path(__file__).parents[2] / "references" / "assets"))

    pr = sub.add_parser("process")
    pr.set_defaults(fn=cmd_process)
    pr.add_argument("image")
    pr.add_argument("--view", required=True, choices=["front", "top", "side"])
    pr.add_argument("--out", required=True)
    pr.add_argument("--assets",
                    default=str(Path(__file__).parents[2] / "references" / "assets"))
    pr.add_argument("--px-per-mm", type=float, default=4.0, dest="px_per_mm")
    pr.add_argument("--camera", default=None, help="camera.yaml to undistort before detection")
    pr.add_argument("--segmenter", choices=["classical", "sam"], default="classical",
                    help="object segmentation backend ('sam' needs the [sam] extra)")

    asm = sub.add_parser("assemble")
    asm.set_defaults(fn=cmd_assemble)
    asm.add_argument("front")
    asm.add_argument("top")
    asm.add_argument("side")
    asm.add_argument("--out", required=True)
    asm.add_argument("--title", default="Part")
    asm.add_argument("--bundle", default=None)

    args = p.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()
