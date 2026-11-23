# MPU-6050 Posture Evaluation Plan

For Issue #7 (M4). An MPU-6050 module is in stock. This plan fixes
the evaluation method and the go/no-go criteria **before** any
measurement, per CONTRIBUTING (criteria are not loosened after
seeing results). Stabilisation is optional: the robot must first
walk acceptably without it (Issues #5, #6).

## Electrical

- The MPU-6050 shares the I2C bus with the PCA9685. Addresses do
  not conflict: PCA9685 at 0x40, MPU-6050 at 0x68 (AD0 low).
- Most MPU-6050 breakout modules regulate to 3.3 V and tolerate a
  5 V I2C pull-up rail, but confirm the actual module before wiring:
  if it is a bare 3.3 V part, add level shifting or pull the bus up
  to 3.3 V (the PCA9685 accepts 3.3 V logic). Record the module
  variant here when wired.
- Servo leads are a noise source: keep the I2C run short and away
  from the PCA9685 output bundle.

## Mounting

- On the top deck, as close to the body centre as the wiring
  allows, axes aligned with the body (module x forward). Note the
  exact position/orientation in this file when mounted; the filter
  assumes the sensor frame matches the body frame.
- Rigid mount (no foam tape): soft mounting phase-shifts the very
  vibration we need to observe.

## Phases

1. **Sense and log only.** Firmware reads accel/gyro at the gait
   update rate (every 20 ms), runs a complementary filter for
   pitch/roll, and streams the estimates over serial on request.
   No control changes. Verify on the bench: static tilt table
   checks at 0 / +/-15 / +/-30 deg must read within +/-3 deg.
2. **Baseline comparison.** With the robot trotting on the floor
   (the tuned gait from Issue #6), log pitch/roll for 10 runs of
   about 1 m. Record RMS and peak pitch/roll, plus falls/veers
   (shared with the Issue #6 tuning log).
3. **Compensation trial (only if Phase 2 shows a problem worth
   fixing).** Open-loop body-levelling: offset each leg's
   hip_height target by the measured pitch/roll (small gains,
   capped at +/-8 mm). Same 10-run protocol.

## Go / no-go criteria (fixed in advance)

- **Adopt sensing permanently** if Phase 1 passes the tilt checks:
  the logs are useful for tuning regardless of stabilisation.
- **Adopt compensation** only if, versus the Phase 2 baseline, it
  reduces pitch+roll RMS by at least 30% **and** does not increase
  falls/veers over the 10 runs. Otherwise keep sensing for logging
  only and close Issue #7 as evaluated/no-adopt.
- **Stop condition:** if I2C traffic to the MPU-6050 measurably
  disturbs servo updates (jitter or missed steps on the bench),
  drop to logging at 5 Hz; if it still disturbs, abandon the shared
  bus and close as no-adopt with the reason recorded.

## Resource budget

Firmware currently uses 38% flash / 31% RAM (after the serial
calibration mode). The MPU-6050 driver and filter must fit in
roughly +15% flash; if not, logging moves to a build-time option
rather than growing the default firmware.
