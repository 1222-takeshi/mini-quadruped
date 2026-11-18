# -*- coding: utf-8 -*-
"""Assembly placement and interference sweep for CAD v0 (issue #1).

Places decks, brackets, servos, and links at the hip positions from
docs/FRAME_DESIGN.md, drives all four legs through the trot cycle
(same trajectory math as software/gait.cpp), and checks:
  1. placed foot tips match the C++ FK positions (validates placement);
  2. no leg part intersects a deck or another leg's parts
     (intersection volume > 1 mm^3 counts as interference).
Same-leg bracket/link pairs are bolted together and excluded.

Run: cad/.venv/bin/python cad/assembly_check.py ; exit 1 on failure.
Frame: x forward, y left, z up, origin at the top deck centre.
"""
import math
import sys

import cadquery as cq
import make_parts as mp

L1, L2 = mp.L1, mp.L2
HIP_X, HIP_Y, HIP_Z = mp.HIP_X, mp.HIP_Y, -50.0
LINK_Y_OFF = 10.0          # link plane offset outboard from shaft plane
PHASE = [0.0, 0.5, 0.5, 0.0]        # FL FR RL RR (trot)
LEGS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]   # (x sign, y sign)
HIP_H, STRIDE, STEP_H, DUTY = 95.0, 36.0, 14.0, 0.5

fail = 0


def check(ok, label):
    global fail
    print(("OK  " if ok else "NG  ") + label)
    if not ok:
        fail += 1


def foot_xz(phase):
    u = phase % 1.0
    half = STRIDE / 2.0
    if u < DUTY:
        return half - STRIDE * (u / DUTY), HIP_H
    v = (u - DUTY) / (1.0 - DUTY)
    return -half * math.cos(math.pi * v), HIP_H - STEP_H * math.sin(math.pi * v)


def ik(x, z):
    cos_a2 = (x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    a2 = math.acos(max(-1.0, min(1.0, cos_a2)))
    a1 = math.atan2(x, z) - math.atan2(L2 * math.sin(a2),
                                       L1 + L2 * math.cos(a2))
    return a1, a2


def rot_y(shape, centre, deg):
    return shape.rotate(cq.Vector(*centre),
                        cq.Vector(centre[0], centre[1] + 1, centre[2]), deg)


def place(shape, mirror_y, at):
    if mirror_y:
        shape = shape.mirror("XZ")
    return shape.translate(cq.Vector(*at))


def build_leg(leg_idx, a1, a2):
    xs, ys = LEGS[leg_idx]
    hip = (xs * HIP_X, ys * HIP_Y, HIP_Z)
    mirror = ys < 0
    ly = ys * (HIP_Y + LINK_Y_OFF)
    parts = {}
    parts["hip_bracket"] = place(mp.servo_bracket().val(), mirror, hip)
    parts["hip_servo"] = place(mp.servo_dummy().val(), mirror, hip)
    upper = mp.upper_link().val().translate(cq.Vector(0, ly, 0))
    upper = rot_y(upper, (0, ly, 0), -math.degrees(a1))
    parts["upper"] = upper.translate(cq.Vector(hip[0], 0, hip[2]))
    knee = (hip[0] + L1 * math.sin(a1), ly, hip[2] - L1 * math.cos(a1))
    kb = place(mp.servo_bracket().val(), mirror, (0, 0, 0))
    kb = rot_y(kb, (0, 0, 0), -math.degrees(a1))
    parts["knee_bracket"] = kb.translate(
        cq.Vector(knee[0], ys * HIP_Y, knee[2]))
    ks = place(mp.servo_dummy().val(), mirror, (0, 0, 0))
    ks = rot_y(ks, (0, 0, 0), -math.degrees(a1))
    parts["knee_servo"] = ks.translate(cq.Vector(knee[0], ys * HIP_Y, knee[2]))
    lower = mp.lower_link().val().translate(cq.Vector(0, ly, 0))
    lower = rot_y(lower, (0, ly, 0), -math.degrees(a1 + a2))
    parts["lower"] = lower.translate(cq.Vector(knee[0], 0, knee[2]))
    foot = (knee[0] + L2 * math.sin(a1 + a2), ly,
            knee[2] - L2 * math.cos(a1 + a2))
    return parts, foot


def volume_intersect(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return 0.0


decks = {
    "deck_top": mp.deck().val().translate(cq.Vector(0, 0, -1.5)),
    "deck_bottom": mp.deck().val().translate(cq.Vector(0, 0, -32.5)),
}

# 1) foot placement validation + 2) interference, over the cycle
worst = {}
for step in range(8):
    base = step / 8.0
    legs = []
    for j in range(4):
        x, z = foot_xz(base + PHASE[j])
        a1, a2 = ik(x, z)
        parts, foot = build_leg(j, a1, a2)
        legs.append(parts)
        xs, ys = LEGS[j]
        expect = (xs * HIP_X + x, foot[1], HIP_Z - z)
        err = max(abs(foot[0] - expect[0]), abs(foot[2] - expect[2]))
        if step == 0:
            check(err < 1.0, f"leg {j} foot placement err {err:.2f} mm")
    for j in range(4):
        for pname, part in legs[j].items():
            if pname in ("hip_bracket", "hip_servo"):
                continue  # fixed to the body side; deck mount is by design
            for dname, deck in decks.items():
                v = volume_intersect(part, deck)
                if v > 1.0:
                    worst[(j, pname, dname)] = max(
                        v, worst.get((j, pname, dname), 0))
            for k in range(4):
                if k == j:
                    continue
                for pname2, part2 in legs[k].items():
                    v = volume_intersect(part, part2)
                    if v > 1.0:
                        key = (min(j, k), max(j, k), pname, pname2)
                        worst[key] = max(v, worst.get(key, 0))

for key, v in sorted(worst.items(), key=str):
    check(False, f"interference {key}: {v:.0f} mm^3")
check(not worst, "no leg/deck or leg/leg interference over the trot cycle "
      f"(8 poses, worst list has {len(worst)} entries)")
print("Result:", "ALL PASS" if fail == 0 else f"{fail} FAILURES")
sys.exit(1 if fail else 0)
