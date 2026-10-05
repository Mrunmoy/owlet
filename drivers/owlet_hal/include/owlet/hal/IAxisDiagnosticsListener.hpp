#pragma once

#include "owlet/hal/IAxisDiagnostics.hpp"

namespace owlet::hal
{

/// Told when what a motor driver reports about itself changes, so nothing
/// has to keep reading the driver to find out.
class IAxisDiagnosticsListener
{
public:
    virtual ~IAxisDiagnosticsListener() = default;

    /// Called when the driver starts or stops answering, when its fault
    /// bits change, or when its stall indication changes. Not called for a
    /// change in the load figure alone.
    /// @param now The driver's state after the change.
    virtual void onDiagnosticsChanged(const AxisDiagnostics& now) = 0;
};

} // namespace owlet::hal
