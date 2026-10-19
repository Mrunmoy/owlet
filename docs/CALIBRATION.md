# Calibration and hardware record

How the head's axes are defined and how to set their limits, followed by what has been measured on the reference head. Calibration is stored on the device (in the Pico's flash) and changed from the PC with `owlet cal`; the factory values are in `apps/pico-owlet/board.hpp`.

The head has no position feedback: `$POS` and the hardware tests report what was *commanded*, counted step by step. Wiring and mechanical faults, and lost steps, only show from outside, by eye or through a camera on the head.

## What defines an axis

Three things define each axis:

| What | Where it lives | Reference head |
|------|----------------|----------------|
| Steps per degree | Build constants in `apps/pico-owlet/board.hpp` (full steps, microsteps, belt ratio) | 200 × 16 × 3 / 360 = 26.67 |
| Direction | `kPanReversed` / `kTiltReversed` in the same file | both `true`: pan + is the camera's right, tilt + is up (confirmed by eye) |
| Soft limits | Factory values in `board.hpp`; change on the device with `owlet cal <axis> --limits MIN MAX` and `owlet cal save` | factory pan ±90, tilt ±60; saved on the reference head: pan ±180, tilt ±135 (2026-10-09) |

On the wire the head reports the record's three drive numbers as `0/0/0`; only the limits are adjustable, up to the travel compiled into `board.hpp`: pan ±180 and tilt ±135 since 2026-10-09 (pan ±170 and tilt ±90 before). A setting the firmware accepts is not a measured mechanical limit, so check the cradle and the cables on the assembled rig before saving a wider range.

**Tilt zero on the reference head is straight up** (since 2026-10-09). The motor signs are unchanged, so negative tilt goes forward and down: −90 looks forward and level, −135 is 45° below level in front, and +135 is 135° backwards from upright. Name the reference while the head is disabled (`owlet enable <pan> <tilt>` from disabled assigns it without moving); enabling with a pose while already powered moves the head to that pose instead.

### Home

The head has no position sensor. **Whatever pose it is in when it powers up (or is flashed) is taken as 0, 0.** So before powering on, centre both axes by hand; a pen line across the base and yoke, and across the yoke arm and cradle, makes that quick. `owlet enable` then holds that pose. If the head is not centred, either centre it with the motors disabled, or tell it where it is: `owlet disable`, then `owlet enable <pan> <tilt>`.

### Finding the limits

1. Put the head in its zero pose by hand, power on, `owlet enable`, `owlet speed 10`.
2. Widen the limit you are exploring, up to the compiled travel: `owlet cal pan --limits -180 180` (not saved).
3. `owlet nudge` towards an end in small steps. The end is where a cable goes tight or something would touch, minus a margin.
4. Set and keep the result: `owlet cal pan --limits MIN MAX`, `owlet cal save`. To make it the factory value too, edit `kPanAxis` / `kTiltAxis` in `board.hpp`.

### Measured, 2026-10-05

Checked by eye at 10 °/s (pan) and 30 °/s (tilt), on a 5 V supply, with the driver potentiometers as shipped:

- **Pan:** +90, 0, −90, 0. Each sweep a true quarter turn, returning to the centre mark both times, so 3:1 and 1/16 are right and no steps were lost.
- **Tilt:** +60, 0, −60, 0, clean. ±60 is now the factory limit.
- **Directions:** with both flags `false`, pan + turned the head to the camera's left and tilt + pointed it down. Both flags were set `true`, and a repeat move to +25, +25 was confirmed: camera's right, and up. Pan + is the camera's *own* right, which is the left of someone standing in front of it.
- Not checked: anything beyond ±90 pan or ±60 tilt; behaviour above 30 °/s.

### Speed and acceleration on 12 V, 2026-10-05

Supply: a 12 V 2 A adapter (13.3 V measured at light load). Driver potentiometers still as shipped; they are under the heatsinks and were never adjusted.

| Test | Result |
|------|--------|
| Supply current, both motors holding | 0.23 A |
| Supply current, both axes moving hard | 0.81 A peak |
| Pan ±80° sweeps at 150, 180, 240, 300, 360 °/s (960 °/s²), twice | clean, back on the centre mark |
| Tilt ±50° sweeps at 60 to 360 °/s (960 °/s²) | clean, back level |
| Quick reversals at 1920 °/s²: pan ±40°, tilt ±30° | clean |
| Both axes together: corner to corner (±60°, ±40°) at 120, 240, 360 °/s, then quick diagonal reversals, at 1920 °/s² | clean, back on both marks |

Motors and drivers stayed cool throughout. The only noise was the printed parts resonating.

**Limits set in the firmware:** 240 °/s and 1440 °/s², about two thirds of what ran clean. Neither axis was pushed to the point of losing steps, so the true limits are higher and unknown.

**Calibration storage:** a limits change saved with `owlet cal save` was still there after a reset (`picotool reboot`), and `owlet cal reset` restored the factory values. Not tried across a full power cycle.

### Re-check from the Raspberry Pi, 2026-10-09

The Pico was moved to the Pi (`pi3b`) and the motors re-plugged. Both drivers answered at 458 mA with no faults. Pan + = camera's right and tilt + = up, unchanged. `owlet selftest --yes`: 16 passed, 0 failed, 1 skipped (home switches): pan ±45° and tilt ±30° at 60, 120 and 240 °/s, and both together at 120 °/s. Back on both marks by eye. The stored limits were still pan ±90, tilt ±60 at this point; the wider range below came later the same day.

### Wider range, 2026-10-09

With the compiled travel raised to pan ±180 and tilt ±135, each axis was swept on its own with the owner watching, through camera-tracking's `Tracker`-backed jog at 5 °/s, slowing to 1 °/s for the last 5° of each leg.

- **Pan:** +90, +170, +180, 0, −90, −170, −180, 0 with tilt level. The owner confirmed both ends smooth and clear, with no contact or cable strain; the view returned to the same place after each return to zero.
- **Tilt,** from the owner's confirmed upright zero (assigned while disabled): 0, −90, −120, −135, 0, +90, +120, +135, 0. Both ends were reached; at −90 the view was level and upright, and at +90 the rear view was inverted, as expected past the top. The head came back to the starting ceiling view at a counted (0, −0.2).
- Drivers afterwards: no active faults, no stall flags, 458 mA. No new trouble codes (one older pan code 21, detail 16, from before the reference reset, was left in place).
- **Saved:** pan ±180, tilt ±135. Motors left disabled.
- **Not confirmed:** the owner's own look at tilt cradle and cable clearance at ±135. These are watched sweeps of counted steps, not angle measurements.

The opt-in check `tools/python/tests/test_wide_calibration_hardware.py` (`OWLET_WIDE_CAL_HW=1`, head disabled, camtrack stopped) proves the firmware accepts ±180/±135 and refuses 0.1° beyond; it restores the original limits and never moves the head.
