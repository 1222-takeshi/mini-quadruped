# Plan

## Concept

- 8 servos, two per leg (hip pitch, knee). ~15 cm class, lightweight,
  printed frame.
- Trot gait first (diagonal pairs FL+RR / FR+RL, phase offset 0.5).
  Foot trajectory: straight line during stance, ellipse during swing.
- The C++ reference implementation (leg_ik / gait) is the source of truth
  for parameters; the Nano firmware consumes the same parameters.

## Control

Arduino Nano (clone) computes joint targets and writes them to a PCA9685
over I2C; the PCA9685 generates all PWM, so timing stays stable even with
the Nano's limited compute.

## Stages

1. Kinematics and gait with numerical verification (done, off-hardware)
2. Frame design and printing (parts within 150 mm per side)
3. Firmware bring-up and per-servo calibration
4. Bench test, standing, trot walking
5. Gait tuning; optional IMU-based stabilization

## Constraints

- Metal-gear servos (MG90S) for the legs; plastic-gear 9 g servos are too
  fragile for leg loads.
- Keep the body light: the servos' torque budget is the design limit.
