#pragma once

#include <cstdint>

namespace owlet::hal
{

/// A single PWM output configured as a servo channel. The underlying period
/// is a property of the driver (50 Hz for hobby servos); callers only provide a
/// pulse width in microseconds.
//
/// owlet::Servo wraps this with calibration + velocity
/// clamping. The PCA9685 driver exposes a Channel proxy that implements this.
class IServoChannel
{
public:
    virtual ~IServoChannel() = default;

    /// Set the active pulse width in microseconds (typically 500..2500 for a servo).
    /// Out-of-range values and bus errors both return false.
    [[nodiscard]] virtual bool setPulseUs(uint16_t pulseUs) = 0;

    /// Stop pulsing (full-OFF). Servo goes limp / loses holding torque.
    [[nodiscard]] virtual bool setIdle() = 0;
};

} // namespace owlet::hal
