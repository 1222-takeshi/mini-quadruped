# Bring-up, Calibration, and Bench Test Procedures

Step-by-step procedures and checklists for Issues #3 (firmware
bring-up), #4 (per-servo calibration), and #5 (bench gait test and
first steps). Written so the work can start the day the parts arrive,
with no design decisions left open. Record results in the tables in
this file (or copy them into the issue) as each step is done.

## Power and safety rules (all stages)

- Servo power comes from the UBEC (2S LiPo input, 5-6 V output) into
  the PCA9685 V+ terminal. **Never** power servos from the Nano's
  5 V pin or from USB.
- Nano (USB or its own regulated input), PCA9685 logic VCC, and the
  UBEC output share a common ground. A missing common ground is the
  most common cause of random servo motion.
- First power-up of any new wiring: bench supply or UBEC with the
  robot unloaded (no legs fitted, or robot suspended), one servo
  connected at a time.
- If the Nano resets when servos move, stop: that is servo-rail
  brownout, not a software bug. Check UBEC current rating and wiring
  before continuing.
- Keep the battery disconnected while wiring. LiPo handling per the
  pack maker's instructions; never leave a charging pack unattended.

## Stage A - Nano firmware bring-up (Issue #3)

Toolchain (command line, no IDE required):

```sh
arduino-cli core install arduino:avr
arduino-cli lib install "Adafruit PWM Servo Driver Library"
arduino-cli compile --fqbn arduino:avr:nano software/firmware/quadruped_nano
arduino-cli upload -p <port> --fqbn arduino:avr:nano software/firmware/quadruped_nano
```

Clone Nano boards often use the old bootloader; if upload fails with
sync errors, retry with `--fqbn arduino:avr:nano:cpu=atmega328old`
and record which variant this board needs below.

Checklist - acceptance is "PWM output on all 8 channels":

- [x] Sketch compiles with no errors - done 2026-11-11 (arduino-cli
      1.6.0, arduino:avr 1.8.8, Adafruit PWM Servo Driver Library
      3.0.3; flash 8186 bytes, RAM 304 bytes). Re-check after edits.
- [ ] Board flashes; note bootloader variant that worked: ________
- [ ] I2C scan finds the PCA9685 at 0x40
- [ ] With a single test servo (or servo tester / oscilloscope if
      available), channels 0-7 each produce a 50 Hz pulse that
      sweeps when the commanded angle changes
- [ ] Pulse width at 0 / 90 / 180 deg is approx. 500 / 1500 / 2500 us
- [ ] No Nano reset or servo jitter with all 8 channels driven
      (servos unloaded is fine at this stage)

## Stage B - Per-servo calibration (Issue #4)

Reference neutral pose (from the C++ model, foot at x=0, z=95 mm):
hip joint -33.5 deg, knee joint +60.8 deg, i.e. servo commands of
hip 56.5 deg and knee 125.8 deg with the default knee offset of
-25 deg, before per-leg direction/offset corrections.

Procedure, repeated per leg (FL, FR, RL, RR), servos fitted to the
frame but the robot suspended:

1. Disconnect the horn from the link. Command the servo to its
   centre (90 deg) via the firmware's calibration path (or a servo
   tester set to 1500 us).
2. Fit the horn/link so the leg points as close as possible to the
   reference neutral pose; this minimises the offset to record.
3. Command the reference neutral pose. Measure the remaining error:
   if the leg moves opposite to the model when the phase advances,
   set that joint's direction to -1; then set the offset so the
   commanded pose matches the model pose.
4. Sweep the joint +/-20 deg around neutral and confirm the motion
   direction and smoothness, then record the values in the table.
5. Enter the final values in BOTH places that must stay in sync:
   `software/gait.hpp` (`GaitConfig`) and
   `software/firmware/quadruped_nano/quadruped_nano.ino`. Re-run `make test` after
   editing the C++ config.

Recording table (initial values: dir +1, hip offset 0, knee -25):

| Leg | Hip dir | Hip offset | Knee dir | Knee offset | Checked |
|---|---|---|---|---|---|
| FL | | | | | |
| FR | | | | | |
| RL | | | | | |
| RR | | | | | |

Acceptance (Issue #4): with legs attached, the commanded neutral
pose matches the reference pose on all four legs.

## Stage C - Bench gait test and first steps (Issue #5)

Keep the firmware's slow default cycle (1600 ms) for all first runs.

1. Legs in the air (robot suspended, feet free):
   - [ ] Diagonal pairs move together (FL+RR, FR+RL), pairs alternate
   - [ ] No servo stalls, buzzes continuously, or runs hot for
         10 s of continuous trot (the Issue #5 acceptance run)
   - [ ] No link or bracket interferes at any point of the cycle
2. Standing on the floor, power on, gait paused at neutral:
   - [ ] Robot stands level without legs splaying or collapsing
   - [ ] Battery/UBEC wiring is not pinched by any leg sweep
3. First steps on the floor:
   - [ ] Continuous trot for at least 10 s (acceptance), record
         distance and any veer for Issue #6
   - [ ] Stop conditions: servo stall, frame crack, repeated Nano
         reset, battery or UBEC hot to the touch

Observation log (one row per run):

| Date | Stage | Cycle (ms) | Result | Notes |
|---|---|---|---|---|
| | | | | |

## Troubleshooting quick reference

| Symptom | First check |
|---|---|
| Nano resets when servos move | Servo rail: UBEC rating, separate servo supply, common ground |
| All servos jitter | Common ground; PCA9685 VCC from Nano 5 V (logic) not from V+ |
| One leg moves in reverse | That leg's dir sign (Stage B), not the gait code |
| One joint never reaches pose | Horn fitted one spline tooth off; re-centre per Stage B step 1-2 |
| Upload fails with sync errors | Clone old-bootloader variant (Stage A) |
| Servo buzzes at neutral | Load too high or offset pushing past a mechanical stop; re-check Stage B |
