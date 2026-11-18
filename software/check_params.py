#!/usr/bin/env python3
"""Check that the Nano firmware constants match the C++ reference.

The C++ code (leg_ik.hpp / gait.hpp) is the source of truth for gait
parameters; the firmware (quadruped_nano.ino) mirrors them by hand.
This script fails (exit 1) if any mirrored value drifts.

Run: python3 check_params.py   (or: make check-params)
"""
import re
import sys

HPP_GEO = "leg_ik.hpp"
HPP_GAIT = "gait.hpp"
INO = "firmware/quadruped_nano/quadruped_nano.ino"


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def num(text, pattern):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else None


def arr(text, pattern):
    m = re.search(pattern, text)
    if not m:
        return None
    return [float(v) for v in m.group(1).split(",")]


failures = 0


def check(label, cpp, ino):
    global failures
    ok = cpp is not None and ino is not None and cpp == ino
    print(f"    {'OK' if ok else 'NG'}  {label}: C++={cpp} firmware={ino}")
    if not ok:
        failures += 1


geo = read(HPP_GEO)
gait = read(HPP_GAIT)
ino = read(INO)

check("L1 [mm]", num(geo, r"l1 = ([\d.]+)"), num(ino, r"L1 = ([\d.]+)"))
check("L2 [mm]", num(geo, r"l2 = ([\d.]+)"), num(ino, r"L2 = ([\d.]+)"))
check("hip_height", num(gait, r"hip_height = ([\d.]+)"),
      num(ino, r"HIP_HEIGHT = ([\d.]+)"))
check("stride", num(gait, r"stride = ([\d.]+)"), num(ino, r"STRIDE = ([\d.]+)"))
check("step_height", num(gait, r"step_height = ([\d.]+)"),
      num(ino, r"STEP_HEIGHT = ([\d.]+)"))
check("duty", num(gait, r"duty = ([\d.]+)"), num(ino, r"DUTY = ([\d.]+)"))
check("servo_center", num(gait, r"servo_center = ([\d.]+)"),
      num(ino, r"SERVO_CENTER = ([\d.]+)"))

cpp_phase = arr(gait, r"phase_offset = \{([^}]+)\}")
ino_phase = arr(ino, r"PHASE_OFFSET\[4\] = \{([^}]+)\}")
check("phase_offset", cpp_phase, ino_phase)

cpp_knee = arr(gait, r"knee_offset = \{([^}]+)\}")
ino_knee = num(ino, r"KNEE_OFFSET = (-?[\d.]+)")
knee_ok = cpp_knee is not None and ino_knee is not None and \
    all(v == ino_knee for v in cpp_knee)
print(f"    {'OK' if knee_ok else 'NG'}  knee_offset: C++={cpp_knee} "
      f"firmware(single)={ino_knee}")
if not knee_ok:
    failures += 1

for name, cpp_pat, ino_pat in (("hip_dir", r"hip_dir = \{([^}]+)\}",
                                r"HIP_DIR\[4\] = \{([^}]+)\}"),
                               ("knee_dir", r"knee_dir = \{([^}]+)\}",
                                r"KNEE_DIR\[4\] = \{([^}]+)\}"),
                               ("hip_offset", r"hip_offset = \{([^}]+)\}",
                                r"HIP_OFFSET\[4\] = \{([^}]+)\}")):
    check(name, arr(gait, cpp_pat), arr(ino, ino_pat))

print(f"Result: {'ALL PASS' if failures == 0 else f'{failures} MISMATCHES'}")
sys.exit(1 if failures else 0)
