# Servo calibration

*Calibration is stored on the device (NVS) since owlet 0.3.0 and set from the PC with `owlet cal`. Kconfig only provides the factory defaults.*

## What is calibrated

Per axis, stored on the device and changed with `owlet cal` (protocol: `$CAL`, see DESIGN §5.6). The factory defaults come from Kconfig (menu **owlet configuration**):

| Setting | Meaning | Default |
|---------|---------|---------|
| `OWLET_<AXIS>_PULSE_MIN_US`    | pulse at −90°  | 500  |
| `OWLET_<AXIS>_PULSE_CENTER_US` | pulse at 0°    | 1500 |
| `OWLET_<AXIS>_PULSE_MAX_US`    | pulse at +90°  | 2500 |
| `OWLET_<AXIS>_LIMIT_MIN_DEG`   | soft limit, low end  | −90 |
| `OWLET_<AXIS>_LIMIT_MAX_DEG`   | soft limit, high end | +90 |

The mapping is piecewise-linear through center, so the three pulses can be measured independently. To reverse an axis, set `PULSE_MIN_US` above `PULSE_MAX_US`. Soft limits cut the usable range (e.g. tilt −45..+60 when the mount would collide) without changing the mapping.

Measured values are written to the device with `owlet cal …` and persisted with `owlet cal save`. Only if you want different *factory* defaults (e.g. for a batch of identical heads) do you put them in `apps/esp32-owlet/sdkconfig.defaults.local` and rebuild.

## Procedure

1. Remove the horn, or make sure the head can't hit anything.
2. `owlet enable` then `owlet speed 30`.
3. **Center:** `owlet move 0 0`. Re-seat the horn so the head faces forward/level. If you can't get it exact with the spline, note how far off it is.
4. **Endpoints:** `owlet nudge` toward each end in 5° then 1° steps. Watch and listen: the end is the last step before the servo buzzes or stops moving. That angle, mapped through the current calibration, gives the real end pulse.
5. `owlet disable`, then `owlet cal <axis> --pulses MIN CENTER MAX` and/or `--limits MIN MAX` (or `--reverse`). `owlet enable` and check that `move ±90` lands on the physical marks without buzzing. Then `owlet cal save`.
6. Record the numbers below, with the date and servo type.

## Measured values

| Axis | Servo | min µs | center µs | max µs | limits | Date |
|------|-------|--------|-----------|--------|--------|------|
| pan  | MG90S-class | 500 | 1500 | 2500 | **±90** | 2026-10-02 |
| tilt | MG90S-class | 2500 | 1500 | 500 | **±60** (camera hits the base beyond) | 2026-10-02 |

These values are **stored on the reference head** (`owlet cal` → saved in NVS). They are verified to survive a reflash. The firmware's factory values are the generic 500/1500/2500 ±90.

How these were checked: horns were mounted with both servos holding at 0,0, then each axis was stepped by itself in 15° increments at 30°/s while the user watched.

- **Pan:** + turns right. ±30° are symmetric and about 30° by eye. Right reaches a clean +90°. Left visibly reaches −90°, but buzzing was reported there. Both servos also hum steadily whenever they hold (silent when limp, power from the ESP32 5 V pin), so it isn't clear whether −90° presses on the servo's internal end stop. The left limit is stored as **−85°** until the hum is fixed (separate 5 V supply). Then recheck −90° and, if it doesn't strain, run `owlet cal pan --limits -90 90 && owlet cal save`.
- **Tilt:** the servo is mounted mirrored, so + tilted *down* with the defaults. Swapping the endpoints (min 2500, max 500) makes + = up. ±15° are symmetric. A full ±90° (straight up and straight down) is clear of the mechanism.

### 2026-10-02 recheck (camera mounted)

The C270 is now mounted on the head. During camera-tracking bring-up, a webcam check showed that commanding pan moved nothing: the servos were plugged into PCA9685 **channels 1 and 2** instead of 0 and 1. After re-plugging, they were swapped, which the camera also showed: pan moved the image vertically. The final wiring is **pan → channel 0, tilt → channel 1**. Each fix was checked by measuring the image shift with phase correlation (±5° → ~40–70 px along the right axis). The direction was confirmed by eye: pan + = right, tilt + = up, so the stored pulses (tilt reversed) are unchanged.

- Pan: a slow sweep to −85 and +90 was clean at both ends, and the user confirmed ±90 is fine. Stored **−90 .. +90**. This supersedes the −85 note above. Whether that buzzing was a wiring artefact is unknown.
- Tilt: with the camera on, it would hit the base well before −90. Stored **−60 .. +60** at the user's direction. The "full ±90 is clear" note above predates the camera mount.
- **Lesson:** the device has no position feedback, so `$POS` and the hardware test suite only show what was *commanded*. Wiring and mechanical faults are only visible from outside, by eye or with a camera on the head.
- Seen but not yet measured: ±5° moves shift the image by unequal amounts (+5 ≈ 40 px, −5 ≈ 67 px), which suggests ~2° of backlash or deadband depending on approach direction. To be characterised in camera-tracking (CT1).
