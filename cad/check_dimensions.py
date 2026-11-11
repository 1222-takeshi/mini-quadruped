# -*- coding: utf-8 -*-
"""Dimension checks for CAD v0 (see docs/FRAME_DESIGN.md).
Run: cad/.venv/bin/python cad/check_dimensions.py ; exit 1 on failure."""
import sys
import make_parts as mp

fail = 0
def check(ok, label):
    global fail
    print(("OK  " if ok else "NG  ") + label)
    if not ok:
        fail += 1

check(mp.L1 == 50.0 and mp.L2 == 60.0, "link lengths match leg_ik.hpp (50/60)")
for name, fn in mp.PARTS.items():
    bb = fn().val().BoundingBox()
    m = max(bb.xlen, bb.ylen, bb.zlen)
    check(m <= mp.BUILD_MAX, f"{name}: max side {m:.1f} <= {mp.BUILD_MAX:.0f} mm")
# link plates must span at least the axis distance they encode
for name, ln in (("upper_link", mp.L1), ("lower_link", mp.L2)):
    bb = mp.PARTS[name]().val().BoundingBox()
    check(bb.zlen > ln, f"{name}: spans axis distance {ln:.0f} mm (z {bb.zlen:.1f})")
print("Result:", "ALL PASS" if fail == 0 else f"{fail} FAILURES")
sys.exit(1 if fail else 0)
