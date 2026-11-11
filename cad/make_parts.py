# -*- coding: utf-8 -*-
"""
Mini quadruped frame parts (CAD v0). Units are mm throughout.
Parameters are collected at the top; re-running regenerates STEP/STL
into cad/out/. Design inputs: docs/FRAME_DESIGN.md (issue #1).

NOTE: v0 uses the ASSUMED MG90S dimensions from FRAME_DESIGN.md.
Pockets and the horn interface must be re-checked against the real
parts (measurement checklist in FRAME_DESIGN.md) before printing v1.
"""
import os
import cadquery as cq

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# ---------------- parameters (mm) ----------------
BUILD_MAX = 140.0          # per-part design limit (printer: 150)
L1 = 50.0                  # hip axis -> knee axis (FIXED, leg_ik.hpp)
L2 = 60.0                  # knee axis -> foot tip (FIXED, leg_ik.hpp)
# MG90S (ASSUMED, typical published values)
SERVO_L, SERVO_W, SERVO_H = 22.2, 11.8, 31.0   # x, z, y-on-side
POCKET_CLEAR = 0.7
SHAFT_OFF_X = 4.9          # shaft offset from body centre (ASSUMED)
# Deck / layout
DECK_L, DECK_W, DECK_T = 116.0, 78.0, 3.0
DECK_GAP = 28.0            # spacer height (provisional)
HIP_X, HIP_Y = 45.0, 36.0  # hip axis positions (design targets)
WALL = 3.0

# ---------------- parts ----------------

def servo_dummy():
    """MG90S lying on its side, origin at the shaft centre, shaft +y."""
    body = (cq.Workplane("XY")
            .box(SERVO_L, SERVO_H, SERVO_W)
            .translate((-SHAFT_OFF_X, -SERVO_H / 2, 0)))
    shaft = (cq.Workplane("XZ").circle(3.0).extrude(6.0)
             .translate((0, 0, 0)))
    horn = (cq.Workplane("XZ").circle(9.0).extrude(3.0)
            .translate((0, 3.0, 0)))
    return body.union(horn)


def deck():
    p = cq.Workplane("XY").box(DECK_L, DECK_W, DECK_T)
    for sx in (-1, 1):
        for sy in (-1, 1):  # spacer holes
            p = p.cut(cq.Workplane("XY")
                      .circle(1.6).extrude(DECK_T + 2)
                      .translate((sx * (HIP_X + 7), sy * (HIP_Y - 4), -1)))
    for sx in (-1, 1):
        for sy in (-1, 1):  # PCA9685 holes (ASSUMED 56 x 19 pattern)
            p = p.cut(cq.Workplane("XY")
                      .circle(1.6).extrude(DECK_T + 2)
                      .translate((sx * 28.0, sy * 9.5, -1)))
    return p


def servo_bracket():
    """Pocket block holding one servo on its side; used x8 (hip+knee)."""
    bw = SERVO_L + 2 * POCKET_CLEAR + 2 * WALL   # x
    bd = SERVO_H + 2 * POCKET_CLEAR + WALL       # y (open at back)
    bh = SERVO_W + 2 * POCKET_CLEAR + 2 * WALL   # z
    p = cq.Workplane("XY").box(bw, bd, bh).translate((0, -bd / 2 + 6, 0))
    pocket = (cq.Workplane("XY")
              .box(SERVO_L + 2 * POCKET_CLEAR, SERVO_H + 2 * POCKET_CLEAR,
                   SERVO_W + 2 * POCKET_CLEAR)
              .translate((-SHAFT_OFF_X, -SERVO_H / 2, 0)))
    p = p.cut(pocket)
    shaft_hole = (cq.Workplane("XZ").circle(5.0).extrude(bd + 2)
                  .translate((0, 4, 0)))
    p = p.cut(shaft_hole)
    for sx in (-1, 1):  # deck mounting holes (vertical, M3)
        p = p.cut(cq.Workplane("XY").circle(1.6).extrude(bh + 2)
                  .translate((sx * (bw / 2 - 5), 2.0, -bh / 2 - 1)))
    return p


def _link(length, width, tip_pad=False):
    """Plate in the x-z plane, holes at z=0 and z=-length (axis distance)."""
    extra_top, extra_bot = 8.0, 14.0 if tip_pad else 8.0
    total = length + extra_top + extra_bot
    # plate centre placed so the holes land exactly at z=0 and z=-length
    p = (cq.Workplane("XY").box(width, 5.0, total)
         .translate((0, 0, extra_top - total / 2)))
    for z in (0.0, -length):
        p = p.cut(cq.Workplane("XZ").circle(2.6).extrude(7.0)
                  .translate((0, -3.5, z)))
    if tip_pad:
        pad = (cq.Workplane("XY").box(width, 14.0, 4.0)
               .translate((0, 0, -length - 6.0)))
        p = p.union(pad)
    return p


def upper_link():
    return _link(L1, 16.0)


def lower_link():
    return _link(L2, 14.0, tip_pad=True)


def spacer():
    p = cq.Workplane("XY").circle(5.0).extrude(DECK_GAP)
    return p.cut(cq.Workplane("XY").circle(1.6).extrude(DECK_GAP + 2)
                 .translate((0, 0, -1)))


PARTS = {"deck": deck, "servo_bracket": servo_bracket,
         "upper_link": upper_link, "lower_link": lower_link,
         "spacer": spacer, "servo_dummy": servo_dummy}

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for name, fn in PARTS.items():
        part = fn()
        cq.exporters.export(part, os.path.join(OUT, name + ".step"))
        if name != "servo_dummy":
            cq.exporters.export(part, os.path.join(OUT, name + ".stl"))
        bb = part.val().BoundingBox()
        print(f"{name}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
    print("exported to", OUT)
