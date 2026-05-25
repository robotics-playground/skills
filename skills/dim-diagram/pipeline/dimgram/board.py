from __future__ import annotations
from pathlib import Path
import cv2
import numpy as np
import yaml

PAPER_MM = {"A4": (210, 297), "Letter": (216, 279), "A3": (297, 420)}
A4_PRESET = dict(dictionary="DICT_5X5_1000", squares_x=7, squares_y=10,
                 square_mm=24.0, marker_mm=18.0)
PRINT_DPI = 300


def _mm_to_px(mm: float, dpi: int = PRINT_DPI) -> int:
    return int(round(mm / 25.4 * dpi))


def _write_pdf(page_gray: np.ndarray, paper_mm: tuple[float, float], path: Path) -> None:
    """Write the page as a PDF sized to the exact physical paper dimensions, so it
    prints at 100% scale."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    w_in, h_in = paper_mm[0] / 25.4, paper_mm[1] / 25.4
    fig = plt.figure(figsize=(w_in, h_in), dpi=PRINT_DPI)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(page_gray, cmap="gray", vmin=0, vmax=255, aspect="auto")
    ax.axis("off")
    fig.savefig(str(path), dpi=PRINT_DPI)
    plt.close(fig)


def make_board(out_dir: Path, preset: str = "A4", square_mm: float | None = None,
               dictionary: str = "DICT_5X5_1000") -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paper_w, paper_h = PAPER_MM[preset]
    sq = square_mm or A4_PRESET["square_mm"]
    marker_mm = round(sq * 0.75, 1)
    squares_x = int((paper_w - 30) // sq)        # >=15mm margins each side
    squares_y = int((paper_h - 30) // sq)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    bd = cv2.aruco.CharucoBoard((squares_x, squares_y), sq, marker_mm, d)
    board_w_mm, board_h_mm = squares_x * sq, squares_y * sq
    img = bd.generateImage((_mm_to_px(board_w_mm), _mm_to_px(board_h_mm)), marginSize=0)

    # white page with the board centred + a printed 100 mm scale-check line
    page = np.full((_mm_to_px(paper_h), _mm_to_px(paper_w)), 255, np.uint8)
    ox, oy = _mm_to_px((paper_w - board_w_mm) / 2), _mm_to_px((paper_h - board_h_mm) / 2)
    page[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
    y = oy + img.shape[0] + _mm_to_px(8)
    cv2.line(page, (ox, y), (ox + _mm_to_px(100), y), 0, 4)
    cv2.line(page, (ox, y - 12), (ox, y + 12), 0, 4)                    # end ticks
    cv2.line(page, (ox + _mm_to_px(100), y - 12), (ox + _mm_to_px(100), y + 12), 0, 4)
    cv2.putText(page, f"Print at 100% / Actual size. This line = 100.0 mm; square = {sq} mm",
                (ox, y + _mm_to_px(7)), cv2.FONT_HERSHEY_SIMPLEX, 1.5, 0, 3)

    png = out_dir / f"charuco_{preset}.png"
    cv2.imwrite(str(png), page)
    pdf = out_dir / f"charuco_{preset}.pdf"
    _write_pdf(page, (paper_w, paper_h), pdf)
    cfg = dict(dictionary=dictionary, squares_x=squares_x, squares_y=squares_y,
               square_mm=sq, marker_mm=marker_mm, paper=preset)
    (out_dir / "board.yaml").write_text(yaml.safe_dump(cfg))
    return dict(**cfg, board_w_mm=board_w_mm, board_h_mm=board_h_mm,
                png=str(png), pdf=str(pdf))
