# Roadmap

Development follows the process in CONTRIBUTING.md: roadmap -> issues ->
TDD (tests first) -> small PRs (roughly 200 lines or less) -> AI review ->
merge.

## M0 - Foundations (done)

- [x] Leg IK and trot gait generator in C++ with numerical verification
- [x] Arduino Nano firmware skeleton (PCA9685, shared gait parameters)

## M1 - Frame design and printing

- [ ] Frame and leg design within the Adventurer 3 build volume
  (150 x 150 x 150 mm); check servo horn dimensions against the real parts
- [ ] Print and dry-fit with servos

## M2 - Firmware bring-up

- [ ] Compile and flash the Nano firmware
- [ ] Per-servo direction/offset calibration procedure and values

## M3 - Bench and first steps

- [ ] Gait test with legs in the air (no load)
- [ ] Standing, then trot walking on the floor

## M4 - Tuning

- [ ] Gait parameter tuning (stride, step height, period)
- [ ] Optional: MPU-6050 posture sensing and stabilization evaluation
