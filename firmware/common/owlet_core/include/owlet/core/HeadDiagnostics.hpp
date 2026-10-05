#pragma once

#include "owlet/core/IDiagnostics.hpp"
#include "owlet/hal/IAxisDiagnostics.hpp"

namespace owlet::core
{

/// IDiagnostics over one source per axis. Either may be null: that axis then
/// reports nothing available.
class HeadDiagnostics final : public IDiagnostics
{
public:
    HeadDiagnostics(const ::owlet::hal::IAxisDiagnostics* pan,
                    const ::owlet::hal::IAxisDiagnostics* tilt)
        : m_pan(pan), m_tilt(tilt)
    {
    }

    [[nodiscard]] ::owlet::hal::AxisDiagnostics read(Axis axis) const override
    {
        const ::owlet::hal::IAxisDiagnostics* source = (axis == Axis::Pan) ? m_pan : m_tilt;
        return (source != nullptr) ? source->diagnostics() : ::owlet::hal::AxisDiagnostics{};
    }

private:
    const ::owlet::hal::IAxisDiagnostics* m_pan;
    const ::owlet::hal::IAxisDiagnostics* m_tilt;
};

} // namespace owlet::core
