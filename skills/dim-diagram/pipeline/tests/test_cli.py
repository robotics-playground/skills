import subprocess
import sys
from pathlib import Path
import cv2
import numpy as np
import yaml
from dimgram.board import make_board


def _render_scene(board, cfg, mm_per_px=0.25):
    """Board on the left + a known 100 x 60 mm rectangle on the right, at a known scale."""
    px = lambda mm: int(round(mm / mm_per_px))
    bw, bh = cfg["squares_x"] * cfg["square_mm"], cfg["squares_y"] * cfg["square_mm"]
    m = px(20)
    img = np.full((m * 2 + px(bh), m * 3 + px(bw) + px(100)), 255, np.uint8)
    img[m:m + px(bh), m:m + px(bw)] = board.generateImage((px(bw), px(bh)))
    cv2.rectangle(img, (m * 2 + px(bw), m), (m * 2 + px(bw) + px(100), m + px(60)), 0, -1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_cli_process_then_assemble(tmp_path):
    asset_dir = tmp_path / "assets"
    make_board(asset_dir, preset="A4")
    cfg = yaml.safe_load((asset_dir / "board.yaml").read_text())
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["dictionary"]))
    board = cv2.aruco.CharucoBoard((cfg["squares_x"], cfg["squares_y"]),
                                   cfg["square_mm"], cfg["marker_mm"], d)
    img = _render_scene(board, cfg)
    for v in ("front", "top", "side"):
        cv2.imwrite(str(tmp_path / f"{v}.png"), img)

    for v in ("front", "top", "side"):
        r = subprocess.run([sys.executable, "-m", "dimgram.cli", "process",
                            str(tmp_path / f"{v}.png"), "--view", v,
                            "--assets", str(asset_dir),
                            "--out", str(tmp_path / f"{v}.json")],
                           capture_output=True, text=True)
        assert r.returncode == 0, r.stderr

    r = subprocess.run([sys.executable, "-m", "dimgram.cli", "assemble",
                        str(tmp_path / "front.json"), str(tmp_path / "top.json"),
                        str(tmp_path / "side.json"), "--out", str(tmp_path / "drawing"),
                        "--bundle", str(tmp_path / "bundle")],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "drawing.svg").exists()
    assert (tmp_path / "bundle" / "front.json").exists()
