# Print and Assembly Guide

For Issue #2 (print frame and dry-fit servos). Print only CAD v1
(after the measured MG90S dimensions are in `cad/make_parts.py`);
v0 output is for layout checks only. Servo and board handling during
assembly follows the power rules in `docs/BRINGUP.md`.

## Print list

Generate with `cad/make_parts.py` (see `cad/README.md`). All parts
fit the Adventurer 3 (150 mm) with margin; suggested grouping:

| Part | Qty | Batch suggestion |
|---|---|---|
| deck | 2 | one plate, both decks |
| servo_bracket | 8 | two plates of 4 |
| upper_link | 4 | with lower links |
| lower_link | 4 | with upper links |
| spacer | 4 | with any batch (small) |

Starting print settings: 3 walls, 20-30% infill; decks flat on the
bed; links on their side (layer lines along the link, not across the
bending direction); brackets pocket-up. Record filament type/lot and
any failed prints in the issue.

## Fasteners and parts to have on hand

- MG90S x8 with their included horns and screws (ASSUMED shipped
  with horns - confirm on arrival, see FRAME_DESIGN measurements)
- M3 screws/nuts for deck spacers and bracket mounting (lengths:
  10 mm for brackets, 35 mm or standoffs for the deck gap) -
  confirm against stock before printing day
- PCA9685, Arduino Nano (clone), 2S LiPo pack, UBEC, wiring
- Servo tester (optional, speeds up the dry-fit)

## Assembly order

1. Dry-fit each servo into a bracket by hand. It must seat without
   force; if a pocket is tight, record the part and the measured
   pocket size in Issue #2 instead of sanding silently - the v1
   generator parameter gets corrected.
2. Fit horns to hip servos at the centre position (use BRINGUP
   Stage B step 1-2: command 90 deg first, then attach), then bolt
   upper links to the horns.
3. Repeat for knee servos at the upper-link ends and lower links.
4. Mount the four hip brackets under the bottom deck corners
   (hip axes at x = +/-45, y = +/-36 mm), checking each leg swings
   freely through its range before tightening fully.
5. Fit spacers and the top deck; route servo leads to the centre
   bay without pinching (leads must not cross a leg sweep).
6. Mount PCA9685 and Nano on the top deck; fit the battery in the
   bay between decks and confirm the deck closes without pressure
   on the pack.
7. Photograph the assembled robot from top/side/front for the
   record, then hand over to BRINGUP Stage A/B.

## Dry-fit acceptance (Issue #2)

- [ ] All 8 servos mount without forcing
- [ ] Every leg swings its full gait range with no contact
      (compare against `cad/assembly_check.py` expectations)
- [ ] Decks close with battery and boards fitted, no pinched leads
- [ ] Interference/tolerance problems found are listed in Issue #2
      with measurements, not just noted as "tight"
