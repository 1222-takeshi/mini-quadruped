# mini-quadruped

A mini quadruped walking robot: 8 metal-gear micro servos (MG90S class),
two per leg, driven by an Arduino Nano (clone) through a PCA9685 PWM
driver. A trot gait comes first. It is a miniature nod to a transformable
hexapod robot from the author's graduate research.

Japanese README: [README.ja.md](README.ja.md)

## Hardware

| Role | Part |
|---|---|
| Legs | MG90S-class servos x8 (hip pitch + knee per leg) |
| PWM driver | PCA9685 16ch (I2C) |
| Controller | Arduino Nano (clone) |
| Frame | 3D printed (FLASHFORGE Adventurer 3, 150 x 150 x 150 mm) |

## Repository layout

- `software/leg_ik.*` - two-link leg inverse/forward kinematics (C++17).
- `software/gait.*` - trot gait generator (diagonal pairs, elliptical
  swing trajectory, straight stance phase).
- `software/tests/` - verification binary: IK round-trip, foot trajectory
  continuity, servo angle ranges, cycle continuity. Run with `make test`.
- `software/firmware/` - Arduino Nano firmware skeleton sharing the gait
  parameters with the C++ reference.
- `docs/PLAN.md` - design plan. `ROADMAP.md` - development roadmap.
- `CONTRIBUTING.md` - development process (roadmap -> issues -> TDD ->
  small PRs -> review -> merge).

## Status

Kinematics and gait are implemented and verified numerically off-hardware
(all checks pass, matching the reference values). Waiting for servos;
frame design and firmware bring-up are next on the roadmap.
