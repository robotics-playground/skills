import yaml
from dimgram.board import make_board, A4_PRESET


def test_make_board_fits_a4(tmp_path):
    out = make_board(tmp_path, preset="A4")
    cfg = yaml.safe_load((tmp_path / "board.yaml").read_text())
    # footprint must fit on A4 (210 x 297 mm) with >= 15 mm margins
    assert cfg["squares_x"] * cfg["square_mm"] <= 210 - 30
    assert cfg["squares_y"] * cfg["square_mm"] <= 297 - 30
    assert (tmp_path / "charuco_A4.png").exists()
    assert (tmp_path / "charuco_A4.pdf").exists()
    assert out["board_w_mm"] == cfg["squares_x"] * cfg["square_mm"]
    assert cfg["dictionary"] == "DICT_5X5_1000"
