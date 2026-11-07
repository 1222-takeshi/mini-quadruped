# Mini Quadruped Robot — Software

Gait computation for a mini quadruped robot (8 micro servos, 2 per leg),
plus the on-robot firmware sketch. The same gait math is used on the PC side
(this C++ code) and on the robot side (Arduino), so behavior verified here
carries over to the hardware.

## Layout

| File | Contents |
|---|---|
| `leg_ik.hpp` / `leg_ik.cpp` | Two-link leg inverse/forward kinematics (hip pitch + knee). Link lengths are parameters (`LegGeometry`). Unreachable targets throw `std::invalid_argument`. |
| `gait.hpp` / `gait.cpp` | Trot gait generator: diagonal leg pairs with 0.5 phase offset, elliptical swing trajectory. Produces one cycle of target angles for 8 servos, with per-leg direction/offset calibration parameters (`GaitConfig`). |
| `tests/test_main.cpp` | Verification binary: FK/IK roundtrips, foot-trajectory continuity, servo angle range and continuity, periodicity, diagonal-pair consistency. |
| `firmware/quadruped_nano.ino` | Firmware skeleton for Arduino Nano + PCA9685. **Not yet compile-verified** (see the comment at the top of the file). |
| `Makefile` | Build and test entry points. |

Coordinate conventions are documented in `leg_ik.hpp`: x = forward,
z = downward, angles measured from the straight-down direction, and the
default configuration has the knee pointing backward.

## Build and verify

Requires a C++17 compiler (g++) and make. No external dependencies.

```sh
cd software
make test     # builds tests/test_main and runs all checks
```

`make test` exits with code 0 when every check passes. Tolerances match the
original numeric verification: foot travel per step <= 8 mm, servo change
per step <= 12 deg, all servo commands within [0, 180] deg.

## Parameters

Link lengths, stride, step height, hip height, steps per cycle, and the
servo calibration values (center, per-leg direction and offset — the knee
offset defaults to -25 deg) all live in `LegGeometry` (`leg_ik.hpp`) and
`GaitConfig` (`gait.hpp`). After changing parameters, re-run `make test`.
On the real robot, the per-leg direction/offset values are tuned by
measurement; the firmware sketch mirrors the same parameter set.
