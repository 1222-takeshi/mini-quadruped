# Frame Design Specification

Design specification for Issue #1 (frame and leg design within the
Adventurer 3 build volume). This document fixes the design inputs, the
part split, and the checks the CAD must pass, so CAD work can proceed
before the servos arrive. Values are labelled:

- **FIXED** - decided by the software (source of truth) or the hardware
  already chosen; the CAD must match these exactly.
- **ASSUMED** - typical published values for the part class. These are
  placeholders and must be replaced by measurements of the real parts
  before the design is finalised (see "Measurements on arrival").

## Design inputs

| Item | Value | Status |
|---|---|---|
| Upper link length L1 (hip axis to knee axis) | 50.0 mm | FIXED (`software/leg_ik.hpp`) |
| Lower link length L2 (knee axis to foot tip) | 60.0 mm | FIXED (`software/leg_ik.hpp`) |
| Hip height (foot z in neutral stance) | 95 mm | FIXED (`software/gait.hpp`) |
| Stride / step height | 36 mm / 14 mm | FIXED (`software/gait.hpp`) |
| Knee configuration | knee points backward (elbow_sign = +1), all legs | FIXED |
| Knee servo offset | -25 deg | FIXED (calibration, Issue #4 may refine) |
| Printer build volume | 150 x 150 x 150 mm (Adventurer 3) | FIXED |
| CAD design limit per part | 140 mm per side (10 mm margin) | FIXED |
| Servos | MG90S class, x8 (2 per leg) | FIXED |
| Controller / driver | Arduino Nano (clone), PCA9685 16ch I2C | FIXED |
| Power | In-stock 2S LiPo battery + UBEC (5-6 V servo rail) | FIXED (owner's stock) |
| Battery pack dimensions | approx. 75 x 35 x 20 mm class | ASSUMED - measure the actual pack |
| PCA9685 board | approx. 62.5 x 25.4 mm, 4 corner holes | ASSUMED - verify against the board in hand |
| Nano (clone) board | approx. 43 x 18 mm | ASSUMED - verify against the board in hand |

Servo power must never be drawn from the Nano's 5 V pin. The UBEC feeds
the PCA9685 V+ rail directly; Nano, PCA9685 logic, and UBEC share a
common ground (see `docs/BRINGUP.md`).

## Leg placement

- Two degrees of freedom per leg (hip pitch, knee). No hip yaw/roll:
  every leg moves in a sagittal plane parallel to the body's sides.
  Turning is handled in software later (Issue #6), not by the frame.
- Hip pitch axes point sideways (laterally), shafts facing outward.
  Left and right brackets are mirror images; the resulting direction
  reversal is absorbed by the per-leg direction calibration in
  Issue #4, not by wiring or frame changes.
- Hip axis positions (design targets): x = +/-45 mm (90 mm apart),
  y = +/-36 mm (72 mm apart, axis planes). With a 36 mm stride the
  front and rear feet stay at least 90 - 36 = 54 mm apart during the
  gait, so legs of 12-16 mm thickness cannot collide.
- Clearance is checked over a joint sweep of +/-35 deg around the
  neutral pose, wider than the gait actually commands (hip joint
  range in the verified gait is about -45 to -22 deg).

## MG90S model (for CAD)

All values in this section are **ASSUMED** from typical MG90S data
(body 22.2 x 11.8 x 31.0 mm, mass 13.4 g, stall torque 1.8 kg-cm at
4.8 V / 2.2 kg-cm at 6.0 V, flange approx. 32-33 mm ear to ear).
Servo pockets use the nominal body size plus 0.6-0.8 mm clearance.
The following are NOT assumed; they must be measured on arrival
because clones vary: horn hole pattern (pitch circle and hole
diameter), flange screw-hole spacing and diameter, spline form.

## Part split

Distinct designs: 6. Total printed pieces: about 18-22. The largest
single dimension is the deck at 116 mm, within the 140 mm limit.

| Part | Qty | Size (mm) | Notes |
|---|---|---|---|
| Deck, top | 1 | 116 x 78 x 3 | Mounting holes for PCA9685 and Nano |
| Deck, bottom | 1 | 116 x 78 x 3 | Battery bay between the decks |
| Hip bracket L / R | 2 + 2 | approx. 40 x 20 x 36 | Mirrored pair; MG90S pocket |
| Upper link | 4 | axis distance exactly 50.0 | Knee servo pocket at the far end |
| Lower link | 4 | axis to tip exactly 60.0 | Tapered, flat foot pad |
| Deck spacer | 4 | height 28 | Or M3 standoffs if available |

Deck separation of 28 mm is provisional: it must clear the PCA9685
with its servo connectors plugged in, and the battery bay must clear
the measured 2S pack height. Both are re-checked at CAD v1.

## Mass and torque budget

| Item | Estimate |
|---|---|
| Servos 8 x 13.4 g | 107 g |
| Nano + PCA9685 + wiring | 50 g |
| 2S battery + UBEC | 80-100 g |
| Printed frame | 60-80 g |
| **Total** | **300-360 g** |

In trot, two legs support the body: about 170 g per supporting leg.
Worst static hip torque at the maximum 18 mm foot offset is about
0.31 kg-cm, well inside the 1.8 kg-cm rating, leaving margin for leg
mass and dynamics. Keep the frame light anyway: the torque budget,
not print strength, is the design limit (`docs/PLAN.md`).

## CAD workflow and verification

- One parametric generator script (`cad/make_parts.py`, CadQuery) with
  every dimension as a named parameter at the top of the file, in the
  same style as the other in-house projects. Generated STEP/STL go to
  `cad/out/` (git-ignored).
- Before a CAD PR is reviewed, a dimension check script verifies:
  1. every part's bounding box is <= 140 mm per side;
  2. upper/lower link axis distances are exactly 50.0 / 60.0 mm;
  3. no interference between legs, links, and decks over the +/-35 deg
     hip/knee sweep around the neutral pose.
- Print guidance (starting point): 3 walls, 20-30% infill for links
  and brackets; decks flat on the bed; links printed on their side so
  layer lines do not cross the bending direction.

## Measurements on arrival (before CAD v1)

Record in this file when the MG90S parts arrive, then regenerate:

- [ ] Servo body L x W x H (3 samples, worst case)
- [ ] Flange: ear-to-ear length, hole spacing, hole diameter
- [ ] Horn: type shipped, hole pitch circle, hole diameter, spline
- [ ] Servo lead length and connector type
- [ ] 2S battery pack L x W x H and lead/connector type
- [ ] PCA9685 and Nano board dimensions and hole positions

## Revision plan

- CAD v0: dummy servo model from the ASSUMED values; decks, brackets,
  and links; dimension checks passing. Proves the layout and split.
- CAD v1: pockets and horn interfaces replaced by the measured
  values; deck separation and battery bay finalised. This is the
  version Issue #2 prints.
