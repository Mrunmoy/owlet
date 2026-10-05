#pragma once

#include "owlet/core/ICalibration.hpp"
#include "owlet/hal/IAxisDiagnostics.hpp"

namespace owlet::core
{

/// What a protocol may ask about the motor drivers. A head with nothing to
/// report still answers, with `available` false.
class IDiagnostics
{
public:
    virtual ~IDiagnostics() = default;

    [[nodiscard]] virtual ::owlet::hal::AxisDiagnostics read(Axis axis) const = 0;
};

} // namespace owlet::core
