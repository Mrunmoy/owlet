# Stepper head: printable mechanism

A belt-driven pan-tilt head for two pancake NEMA 17 motors. Each axis is a 20T motor pulley driving a 60T pulley through a 200 mm closed GT2 belt (3:1), with the output on a 5 mm shaft in two 625ZZ bearings. The camera sits on a cradle whose platform is 10 mm from the tilt axis, so its weight stays near the pivot. The reference head runs with the cradle inverted and the camera on top.

![Assembly](preview_front.png)

## Files

| File | What |
|------|------|
| `pan_tilt.scad` | Parametric OpenSCAD source. Select a part with `-D 'part="yoke"'`; `assembly` shows everything with mock motors, pulleys and belts |
| `stl/base_top.stl` | Base deck and walls; holds the pan motor underneath (exported deck-down for printing) |
| `stl/base_bottom.stl` | Bottom plate with the lower pan bearing and hold-down holes |
| `stl/yoke.stl` | Turntable and two arms; carries the tilt motor and both tilt bearings |
| `stl/cradle.stl` | Camera platform on the tilt shaft (strap slots and a 1/4" screw hole). Grips the shaft with a pinch clamp: one M3 x 25 bolt and nut across the slit in each hub. The earlier set-screw version slipped under the camera's weight and cracked when tightened. The bore is 5.1 mm, nearly rod size; a printed horizontal hole comes out a touch small, so ream it with a 5 mm drill if the rod will not enter |
| `stl/small_parts.stl` | One shaft collar and six 1.5 mm spacers |
| `switch_mounts.scad` | Source for the limit switch mounts (the common 3D-printer endstop board). `-D 'part="pan_bracket"'`, `pan_flag` or `assembly` |
| `stl/pan_bracket.stl` | Pan switch bracket, glued to the base deck; print two, 95° either side of straight ahead |
| `stl/pan_flag.stl` | Flag glued under the yoke that presses the pan switch levers |
| `stl/bushings/` | Sleeves that adapt an 8 mm pulley bore to a 5 mm shaft: a graded set (8.0 to 8.5 mm, rim notches count the size) and one file per size |

No part needs supports in the exported orientation. Use four perimeters and about 30 % infill; PETG is better than PLA near the motors.

## Hardware

| Qty | Part |
|----:|------|
| 2 | NEMA 17 pancake stepper, about 23 mm body, 5 mm shaft |
| 2 | GT2 20T pulley, 5 mm bore |
| 2 | GT2 60T pulley (5 mm bore, or 8 mm with a bushing) |
| 2 | GT2 closed belt, 200 mm, 6 mm wide |
| 4 | 625ZZ bearing (5 × 16 × 5 mm) |
| 2 | 5 mm steel shaft: 80 mm (pan) and 120 mm or longer (tilt) |
| 8 | M3 × 8 screw (motor faces; the motor holes are shallow, so no longer) |
| 6 | M3 × 12 screw (base) |
| 2 | M3 × 8 screw + M3 nut (set screws in the yoke hub and the collar) |
| 2 | M3 × 25 bolt + M3 nut (pinch clamps on the cradle) |

The pulley centre distance, 58.6 mm, follows from the tooth counts and the belt length; the motor mounts are slotted ±3 mm for tension. Grind a small flat on each shaft where a set screw lands.

## Assembly order for the base

1. Bolt the pan motor under the deck, screws loose in the slots, connector towards a side window.
2. Press a bearing into the deck and one into the bottom plate.
3. Slide the pan shaft down through the deck bearing and thread the collar onto it inside the box.
4. Screw on the bottom plate so the shaft end seats in the lower bearing.
5. Push the collar up against the deck bearing and tighten it through the window.
6. On top: spacer, 60T pulley, spacer, then the yoke.
7. Fit the 20T pulley and the belt, slide the motor away to tension it, tighten the motor screws.

## As built (2026-10-05)

The 60T pulleys supplied had an 8 mm bore, not 5 mm. The reference head runs on printed 8.1 mm PLA bushings (`stl/bushings/single_8.1.stl`); 8.15 mm and 8.2 mm would not enter the bore. Metal 5-to-8 mm bushes are the better long-term fit. The model's pulley dimensions (the values marked MEASURE in the source) only position the mock-ups in the assembly view and do not change any printed part.

Pan limit switch mounts are designed (`switch_mounts.scad`) but not yet fitted, and the firmware has them switched off (`kLimitSwitchesFitted` in `apps/pico-owlet/board.hpp`). Tilt mounts are still to come.

On 2026-10-09 the assembled head was swept through pan ±180° and tilt ±135° (tilt zero straight up) and those are its saved limits; see `docs/CALIBRATION.md`.
