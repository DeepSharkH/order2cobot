# Geometry audit: mapper model vs. CAD (casings, new 30 mm finger)

2026-10-27. Each assembled casing was rebuilt from `casings.py` with manifold3d and sliced every
few mm. The real part bodies (`part_shape`) were sliced the same way, and so was
`assets/models/gripper/Gripper_R7_W15to5_L30_T2_C0p2.stl`. Relation counts cover 246,922 relative
placements of two casings (14 casing types, all 4 orientations, ±12 holes, holes not shared).

## Bottom line

- **Casings are not modelled too fat.** Every casing is a straight 20 mm prism with no foot and no
  chamfer. `width_x` × `thickness_y` is the real footprint at every height (STL halves are exactly
  `width_x` × 20 × 2.0).
- **The finger is not modelled too fat either. It is slightly too thin.** 2.0 × 5.0 is the very tip
  only. The finger tapers outward (one flat contact face), so it is 2.26 × 5.23 at 6 mm above the tip.
- **What really is too fat: the LED override.** 6.2 mm is the LED pocket including clearance, and it
  is applied over the whole casing width (14 to 23.7 mm). The LED itself is Ø5.8 (1 mm flange band),
  Ø5.0 dome. It sticks out only within ±2.1 mm of the LED axis, and only between z = 5 and 12.6 mm
  (led_s1). This one fix accounts for all of the relaxation (below).
- **The biggest blocker is not geometry. It is the gripper hardware.** Today the bridge can only
  open fully: each finger moves about 20.3 mm outward instead of 1 mm, which doubles the forbidden
  pairs. At 100 % grip force (what the smart skill uses) the 2 mm finger is overstressed about 4×.

## (a) Casing footprint: `width_x` / `thickness_y`

| casing | width_x (json = CAD) | part body + leads, x span | wasted wall per end |
|---|---|---|---|
| led_s1 / led_s2 | 14.0 | 5.8 / 6.3 | 4.1 / 3.9 |
| resistor_s1 / _s2 / _s4 | 14.4 / 14.4 / 18.2 | 9.3 / 9.3 / 11.4 | 2.6 / 2.6 / 3.4 |
| bjt_s1, capacitor_ceramic_s1 | 14.0 | 6.3, 5.5 | 3.9, 4.3 |
| jumper_s1..s4 | 11.5 / 14.1 / 16.6 / 19.2 | channels at ±pitch | 4.5 (from the 5 mm jumper) |

- The cross-section is the same from z = 0.1 to 19.9 for all 20 casings. There is no narrower foot.
- `thickness_y` = 4.0 is real (two 2.0 halves). Only LED/RGB bodies break through the faces.
  Resistor (Ø2.5), BJT (2.4) and ceramic capacitor (2.9) stay inside.
- The width comes from `casings.py`: `w = max(14.0, ...) + 2 * WALL(2.65)`. This is a **CAD lever,
  not a model error**. With no 14 mm floor and WALL = 1.2, the widths would be: led_s1 8.6,
  resistor_s1/s2 11.5, bjt 10.7, capacitor 8.3. That saves about 2 rows per in-line neighbour, but
  it needs a reprint, so it is outside this change.

## (b) The LED override of 6.2 mm

- 6.2 = 2 × (2.9 + 0.2): the flange **pocket** radius including the 0.2 mm clearance, not the LED.
- What really sticks out of each face (beyond |y| = 2.0), with the LED axis at casing x = 0:

| casing | flange Ø5.8 band (z) | sticks out until z | x extent beyond face |
|---|---|---|---|
| led_s1 | 5.00–6.00 | 12.59 | ±2.1 |
| led_s2 | 5.27–6.27 | 12.86 | ±2.1 |
| led_s4 | 7.81–8.81 | 15.40 | ±2.1 |
| rgb_led_s1 | 6.16–7.16 | 13.75 | ±2.1 |
| rgb_led_s2 | 9.97–10.97 | 17.56 | ±2.1 |

- Above the flange, the dome (Ø5.0) sticks out 0.5 mm per side. The flange (Ø5.8) sticks out 0.9 mm.
- The LED can sit up to 0.2 mm off centre in its pocket. A neighbour face can push it back without
  force, so use the nominal 2.9.
- Real effect on the grid: an LED beside any casing two holes apart (5.08 mm) is **forbidden today**
  (3.1 + 2.0 = 5.10 > 5.08, by 0.02 mm). With the real flange the gap is 0.18 mm.

## (c) The new finger (`Gripper_R7_W15to5_L30_T2_C0p2`)

- The cap is 13.2 × 29.4 mm, z 0..21.5, with a 2.5 mm floor and 2.5 mm walls. Its pocket is
  8.2 × 24.4 mm, i.e. the stock tip plus 0.2 mm clearance per side.
- The finger is 30 mm long (z 0..−30). Its **contact face is flat**: x = +6.6 runs from the cap down
  to the tip. The taper grows only outward, away from the gripped part. Width tapers symmetrically
  from 15 to 5 mm.

| height above tip d (mm) | 0 | 2 | 4 | 6 | 8 | 10 | 12 | 15 | 20 | 25 |
|---|---|---|---|---|---|---|---|---|---|---|
| thickness beyond the contact face t | 2.00 | 2.07 | 2.15 | 2.26 | 2.40 | 2.58 | 2.83 | 3.34 | 4.80 | 7.65 |
| width w | 5.00 | 5.07 | 5.14 | 5.23 | 5.36 | 5.52 | 5.75 | 6.19 | 7.50 | 10.04 |

- **Which part of the finger matters:** every casing is 20 mm tall. With the tip at height z_tip, only
  d ≤ 20 − z_tip is beside a neighbour. The cap is 30 mm above the tip, so it never comes down to
  casing height (that would need z_tip < −10).
- **Plausible grip height:** z_tip = 14 mm, i.e. 6 mm of contact. The earlier grasp with the stock
  tips worked with about 2.6 mm of contact (protoboard-wire skill, 10-26). LEDs need the tip above
  the end of their protrusion, otherwise the finger presses on the LED, 0.5–0.9 mm proud of the
  face. That gives led_s4 ≥ 15.9 (4.1 mm contact). rgb_led_s2 needs ≥ 18.1, leaving only 1.9 mm of
  contact: this casing needs a dome grip or a lower LED.
- **Today's 2.0 × 5.0 is only safe for shallow grips.** Two parallel casings three holes apart
  (7.62 mm) stay independent only while t + opening ≤ 3.62. That allows d ≤ 10.3 with the 1 mm
  opening, or d ≤ 13.8 with 0.5 mm. A finger reaching the board (d = 20) is 4.8 × 7.5.
- **Opening: 1 mm per finger is more than enough.** The contact is flat on flat with no undercut.
  The finger bends under the grip force (PLA assumed, E = 3.5 GPa, 6 mm contact):

| grip force | finger bend | peak stress |
|---|---|---|
| 20 N (Hand-E minimum) | 0.57 mm | 23 MPa |
| 40 N | 1.14 mm | 47 MPa (about PLA yield) |
| 185 N (100 %, what the smart skill uses) | 5.3 mm | 217 MPa (breaks) |

  So command about 1.0 mm per finger at the jaw. The tip itself then travels only about 0.5 mm,
  and **0.5 mm** is the right number for the model.
- **Today's hardware:** the bridge offers only full open or full close. The contact faces sit 2.5 mm
  (+0.2 play) inside the stock tips, so a 4 mm grip has the stock fingers about 9.2 mm apart. A full
  open (50 mm) then moves each tip about **20.3 mm** outward: the contact face ends at ±22.3 mm from
  the casing centre and the finger back at ±24.6 mm.

## (d) Zero margin and tolerances

- The 2.54 mm grid never lets two bodies actually touch. At margin 0 the smallest gaps the grid
  allows are:
  - side by side: 1.08 mm (casing to casing);
  - end to end or T: ≥ 0.18 mm. Only 7 of 301 casing-pair arrangements are below 0.3 mm:
    - led_s1 or capacitor_ceramic_s1 against led_s4 / rgb_led_s1 / rgb_led_s2: 0.18 mm;
    - led_s1 against capacitor_ceramic_s1: 0.18 mm;
    - resistor_s1 against resistor_s4: 0.23 mm.
- Each half is printed lying on its outer face. Thickness comes from the layer stack (±0.05) plus
  the glue line (+0.05–0.1). `width_x` is in the printer's XY plane (±0.1, plus up to 0.2 of
  first-layer squish on the outer face).
  - **Keep body margin 0** if a printed casing measures at most nominal + 0.09 in width.
  - Otherwise a body margin of 0.2 costs only those 7 arrangements one more hole.
- C0p2 is the fit of the cap on the stock fingertip, not a casing tolerance.
  - While the finger is in contact, the casing face fixes where it is, so the play only adds tilt
    over the contact: ≤ 0.15 mm at 6 mm (0.2 mm per side over 19 mm of engagement).
  - It also moves the casing relative to the flange by up to about 0.9 mm (TCP accuracy). That is
    a concern for vision and the spiral search, not for the mapper. Glue or shim the cap anyway.

## (e) Constraints the model has but physics doesn't (and the reverse)

Unnecessary:

1. The LED/RGB 6.2 mm band runs over the full casing width. The real protrusion is only within
   ±2.1 mm of the LED axis.
2. The LED band is 6.2 instead of the real 5.8.
3. A finger is checked against the LED band even when the LED protrusion ends below the fingertip.
   With z_tip 14, led_s1, led_s2 and rgb_led_s1 protrusions (top 12.6–13.75) are never next to a
   finger.
4. The 1 mm opening is the jaw command. The tip only travels about 0.5 mm.

Missing (the model is too thin here):

5. The finger thickens above the tip.
6. Finger tilt from the cap play.
7. Today's full-open release.
8. An LED grip height below the protrusion top: the finger then sits on the LED, not on the
   casing face.

Not modelled and soft: jumper wire loops above 20 mm. Everything else above 20 mm is free, because
only the finger goes below 20 mm.

## Proposed model v2: relaxed but still safe (2.5D)

| item | today | v2 |
|---|---|---|
| casing body | width_x × 4.0, LED/RGB 6.2 full width | width_x × thickness_y (the real casing) |
| LED/RGB protrusion | (in the 6.2 band) | extra box x ±2.1, y ±2.9, with its z range from table (b) |
| grip height (tip) | implicit | `grip_tip_z` = max(14.0, protrusion top + 0.5) |
| finger thickness beyond the face | 2.0 | **2.5**: t(6) = 2.26, + cap tilt 0.15, + casing tolerance 0.1 |
| finger width | 5.0 | **5.5**: w(6) = 5.23, + 0.2 cap play |
| opening (tip travel) | 1.0 | **0.5** (command about 1.0 at the jaw). While only full open exists: **20.3** |
| finger reach from the casing face | 3.0 | 3.0 (unchanged) |
| finger vs. neighbour body | always | casing box: always (all are 20 mm tall). Protrusion box: only if its top > the gripper's `grip_tip_z` |
| body vs. body | 2D | 2D, exact here: all bodies are full-height prisms and the protrusions overlap them in z |
| margin | 0 | body 0 (re-check after measuring printed casings). Finger tolerances are folded into 2.5 / 5.5 |

**Effect over all 246,922 pair placements** (relaxed = forbidden or ordered becomes looser):

| model | forbidden | ordered | independent |
|---|---|---|---|
| today | 24,450 | 5,016 | 217,456 |
| v2 | 22,494 (−8 %) | 3,780 (−25 %) | 220,648 |
| v2 with today's full-open release (20.3) | ≥ 47,930 (×2.0) | ≥ 28,864 (×5.8) | ≤ 170,128 |

- All 3,736 changes from today to v2 involve an LED or RGB casing.
- None goes the stricter way: on the 2.54 / 1.27 grid, a finger of 2.5 × 5.5 with 0.5 opening gives
  exactly the same outcomes for non-LED pairs as 2.0 × 5.0 with 1.0.
- Typical new freedoms:
  - an LED beside any casing two holes apart, staggered so the fingers miss: forbidden → allowed;
  - two LEDs two holes apart, staggered so the fingers miss (≥ 10.16 mm along the leads; the
    bulges alone would need only 4.2): forbidden → allowed;
  - a part three holes from led_s1, led_s2 or rgb_led_s1: ordered → independent. This removes order
    cycles in LED circuits.

## Files and fields that change (same mapping.json contract)

1. `assets/models/component_casings/casings.json`, with `casings.py` emitting the same data. These
   fields are additive and the STLs are unchanged, so nothing is reprinted:
   - LED/RGB casings get `"protrusions": [{"x": [-2.1, 2.1], "y": [-2.9, 2.9], "z": [z0, z1]}]`,
     with z0/z1 from table (b);
   - every casing gets `"grip_min_z": z1 + 0.5`, or 0 for casings without protrusions.
2. `src/circuit_mapper/gripper.json`:
   - `finger_thickness_mm` 2.5, `finger_width_mm` 5.5, `opening_per_finger_mm` 0.5;
   - add `grip_tip_z_mm` 14.0 and optionally `finger_profile_mm` [[d, t, w], ...] from the table in (c);
   - remove `assembled_thickness_overrides_mm`;
   - keep `safety_margin_mm` (the dashboard reads it) and `grip_center_x_mm`.
3. `src/circuit_mapper/geometry.py`:
   - `placement_geometry` keeps `body` as one box (the dashboard draws `_uv(s['body'])`). It adds
     `protrusions` (rotated boxes with z tops) and `grip_z`.
   - `relation` adds protrusion-vs-body and protrusion-vs-protrusion as `forbidden`. Finger vs.
     protrusion counts only if the protrusion top > the gripping part's `grip_z`.
   - Backward compatibility: if a stored mapping's `gripper` still has
     `assembled_thickness_overrides_mm`, honour it, so old mapping.json files keep drawing and
     counting the same. Fall back to defaults when the new keys are missing.
4. `src/circuit_mapper/README.md`: update the geometry section (2.5D rule, grip height, caveats).
   `mapper.py` needs no change, and `mapping.json["gripper"]` carries the new keys automatically.

## Hardware caveats (do not ignore when executing a v2 layout)

- **Release opening.** Only full open or full close exists today. Until the bridge can command a
  Hand-E position (check whether the Robotiq URCap's position move can be called from the
  ur-script node), a v2 layout needs `opening_per_finger_mm` = 20.3 to be executable. A 1 mm
  opening means about 10 counts at about 0.196 mm per count.
- **Grip force.** The new PLA fingers must close at the low end of the force range: 20 N gives
  23 MPa, 40 N is about yield. The smart skill's 100 % (185 N) will break or bend them by about
  5 mm.
- **Finger set.** The 2.x × 5.x numbers are for the new 30 mm fingers only. The stock tips are about
  8 × 24 mm and need different numbers.
- **rgb_led_s2** leaves only 1.9 mm of clean grip below the casing top. Grip it on the dome, or
  lower the LED in the CAD.
