"""Simple 3D model of a Seeed XIAO ESP32-S3 on its own pin headers, for the board STEP.

    uv run --with cadquery python xiao_model.py     # -> 3d/XIAO_ESP32S3_on_pins.step

KiCad 3D-model frame of build_pcb.xiao_fp (mm, z up from the carrier's top copper; y points UP,
i.e. opposite to the footprint's y): rows of 7 pins at x = +-7.62, pitch 2.54 along y, USB-C at
+y here = the footprint's -y (its "USB" silk). Pin headers with a 2.5 mm spacer, XIAO board
21 x 17.8 x 1.2 on top, shield can and USB-C on its upper side. Envelope only (no detail).
"""
import os
from pathlib import Path

import cadquery as cq

OUT = Path(__file__).resolve().parent / "3d"
SPACER, PCB = 2.5, 1.2


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane().box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def main():
    OUT.mkdir(exist_ok=True)
    m = box(-8.9, 8.9, -10.5, 10.5, SPACER, SPACER + PCB)                     # XIAO board
    m = m.union(box(-6.5, 6.5, -9.0, 4.0, SPACER + PCB, SPACER + PCB + 2.4))   # shield can
    m = m.union(box(-4.5, 4.5, 4.4, 11.9, SPACER + PCB, SPACER + PCB + 3.2))    # USB-C, 1.4 past the edge
    for x in (-7.62, 7.62):
        m = m.union(box(x - 1.27, x + 1.27, -8.89, 8.89, 0.0, SPACER))           # header spacer
        for k in range(7):
            y = -7.62 + 2.54 * k
            m = m.union(box(x - 0.32, x + 0.32, y - 0.32, y + 0.32, -3.0, SPACER + PCB + 0.5))
    cq.exporters.export(m, str(OUT / "XIAO_ESP32S3_on_pins.step"))
    print("written", OUT / "XIAO_ESP32S3_on_pins.step")
    os._exit(0)


if __name__ == "__main__":
    main()
