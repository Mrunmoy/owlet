# Servo calibration

*Status (2026-10-01): not yet measured. The firmware uses the nominal 500 / 1500 / 2500 µs for −90 / 0 / +90° on both axes.*

## What is calibrated

Per axis (Kconfig, menu **owlet configuration**):

| Setting | Meaning | Default |
|---------|---------|---------|
| `OWLET_<AXIS>_PULSE_MIN_US`    | pulse at −90°  | 500  |
| `OWLET_<AXIS>_PULSE_CENTER_US` | pulse at 0°    | 1500 |
| `OWLET_<AXIS>_PULSE_MAX_US`    | pulse at +90°  | 2500 |
| `OWLET_<AXIS>_LIMIT_MIN_DEG`   | soft limit, low end  | −90 |
| `OWLET_<AXIS>_LIMIT_MAX_DEG`   | soft limit, high end | +90 |

The mapping is piecewise-linear through center, so the three pulses can be measured independently. To reverse an axis, set `PULSE_MIN_US` above `PULSE_MAX_US`. Soft limits cut the usable range (e.g. tilt −45..+60 when the mount would collide) without changing the mapping.

Put the measured values in `apps/esp32-owlet/sdkconfig.defaults.local` (copy the `.example`), then rebuild and flash.

## Procedure

1. Remove the horn, or make sure the head can't hit anything.
2. `owlet enable` then `owlet speed 30`.
3. **Center:** `owlet move 0 0`. Re-seat the horn so the head faces forward/level. If you can't get it exact with the spline, note how far off it is.
4. **Endpoints:** `owlet nudge` toward each end in 5° then 1° steps. Watch and listen: the end is the last step before the servo buzzes or stops moving. That angle, mapped through the current calibration, gives the real end pulse.
5. Write the pulses into `sdkconfig.defaults.local`, flash, and check that `move ±90` lands on the physical marks without buzzing.
6. Record the numbers below, with the date and servo type.

## Measured values

| Axis | Servo | min µs | center µs | max µs | limits | Date |
|------|-------|--------|-----------|--------|--------|------|
| pan  | —     | —      | —         | —      | —      | —    |
| tilt | —     | —      | —         | —      | —      | —    |
