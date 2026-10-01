#pragma once

#include "owlet/servo/Servo.hpp"

#include <cstdint>

namespace owlet::core
{

enum class Axis : uint8_t
{
    Pan,
    Tilt,
};

// What a protocol may do with the head's calibration (docs/DESIGN.md §5.6).
class ICalibration
{
public:
    enum class Result : uint8_t
    {
        Ok,
        Range,    // values fail validation
        Enabled,  // pulse changes (or reset) need the servos disabled
        Storage,  // persisting failed
    };

    enum class Source : uint8_t
    {
        Factory,   // built-in (Kconfig) values
        Saved,     // loaded from or just written to persistent storage
        Modified,  // changed since the last load/save
    };

    virtual ~ICalibration() = default;

    [[nodiscard]] virtual ::owlet::Servo::Calibration get(Axis axis) const = 0;
    [[nodiscard]] virtual Result set(Axis axis, const ::owlet::Servo::Calibration& cal) = 0;
    [[nodiscard]] virtual Result save() = 0;
    // Restore factory values and erase the saved copy.
    [[nodiscard]] virtual Result reset() = 0;
    [[nodiscard]] virtual Source source() const = 0;
};

} // namespace owlet::core
