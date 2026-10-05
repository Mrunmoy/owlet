#pragma once

#include <cstdint>

namespace owlet::hal
{

/// What a motor driver can tell about itself, in a driver-neutral shape. An
/// axis with nothing to report (a hobby servo, a stepper driver without a
/// data link) leaves `available` false and the rest zero.
struct AxisDiagnostics
{
    enum Fault : uint16_t
    {
        kOverTempWarning = 1U << 0, ///< driver hot, still running
        kOverTemp = 1U << 1,        ///< driver shut down by temperature
        kShortA = 1U << 2,          ///< short on motor coil A
        kShortB = 1U << 3,
        kOpenA = 1U << 4, ///< coil A looks unconnected (only meaningful while moving)
        kOpenB = 1U << 5,
    };

    bool available{false}; ///< the numbers below are live
    uint16_t faults{0};    ///< Fault bits
    bool stalled{false};   ///< load at or beyond the stall threshold while moving
    uint16_t load{0};      ///< driver's load measure; meaning is the driver's (see its docs)
    uint16_t currentMa{0}; ///< motor run current the driver is set to, RMS
};

class IAxisDiagnostics
{
public:
    virtual ~IAxisDiagnostics() = default;

    [[nodiscard]] virtual AxisDiagnostics diagnostics() const = 0;
};

} // namespace owlet::hal
