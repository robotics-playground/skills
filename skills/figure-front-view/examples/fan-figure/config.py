"""Fan figure front view: inputs, palette, parts, stacking order. Artwork pixel coordinates, y down."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
INPUT = ROOT.parent / "input"
WORK = ROOT / "work"
OUT = ROOT

ARTWORK = INPUT / "2026-10-31 19_59_59-_ fan-figure - FreeCAD 1.1.4.png"
PHOTO = INPUT / "2cf1e25b-7702-4baf-a6e8-312ccf582316.jpg"

UP = 6  # working grid = artwork upsampled 6x (sub-pixel masks)

# BGR flat paints sampled on the artwork (see stage1.py)
PALETTE = {
    "bg": (253, 255, 221),
    "blue": (212, 158, 41),
    "white": (254, 254, 254),
    "grey": (222, 222, 222),
    "black": (45, 45, 45),
}

# Provisional scale: figure plate height (helmet top to sole line). 99.9 mm is the value used for the
# same Vinc Studio figure in the skill's worked example; not measured on this print yet.
FIGURE_HEIGHT_MM = 99.9

# back -> front. Body plate behind the rotating arm ring; head plate in front of the ring
# (video t10/t12: hair covers the ring); helmet in front of the face; face inlays on top.
ORDER = ["CRAVAT", "BODY", "ARMS", "HANDS", "FACE", "HELMET",
         "GLASSES", "EYEBROWS", "NOSE", "MOUTH"]
COLOUR = {"CRAVAT": "white", "BODY": "black", "ARMS": "black", "HANDS": "white",
          "FACE": "white", "HELMET": "blue", "GLASSES": "black", "EYEBROWS": "black",
          "NOSE": "black", "MOUTH": "black", "BASE": "black"}
