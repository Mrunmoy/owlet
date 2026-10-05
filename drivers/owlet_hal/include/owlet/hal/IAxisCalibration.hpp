#pragma once

#include <cstdint>

namespace owlet::hal
{

// Per-axis calibration in a motor-neutral shape, so storage, protocols and
// clients never need to know which motor type is fitted.
//
// The three `drive` numbers mean whatever the axis type says they mean. A
// hobby servo uses the pulse widths (µs) at −90°, 0° and +90°; an axis with
// nothing to tune reports zeros and refuses anything else. `driveMin` above
// `driveMax` is the convention for a reversed axis.
struct AxisCalibration
{
    uint16_t driveMin = 0;
    uint16_t driveCenter = 0;
    uint16_t driveMax = 0;
    float minDeg = 0.0f; // soft limits; targets are clamped to these
    float maxDeg = 0.0f;
};

// The calibration face of an axis. Kept apart from IAxis because the motion
// policy never calibrates and the calibrator never moves anything.
class IAxisCalibration
{
public:
    virtual ~IAxisCalibration() = default;

    [[nodiscard]] virtual AxisCalibration calibration() const = 0;

    // What the axis falls back to when given nothing valid.
    [[nodiscard]] virtual AxisCalibration defaultCalibration() const = 0;

    // Would the axis run correctly with `cal`? Must be false for non-finite
    // limits: nothing unvalidated may reach the motor mapping.
    [[nodiscard]] virtual bool acceptsCalibration(const AxisCalibration& cal) const = 0;

    // True if applying `cal` to a powered axis would make it move by itself
    // (a servo given new pulse widths). Such changes need the axis released.
    [[nodiscard]] virtual bool calibrationNeedsRelease(const AxisCalibration& cal) const = 0;

    // Apply an accepted calibration. A powered axis left outside new limits
    // travels back inside at speed(); it never jumps.
    virtual void applyCalibration(const AxisCalibration& cal) = 0;
};

} // namespace owlet::hal
