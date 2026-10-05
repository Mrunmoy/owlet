#pragma once

#include "owlet/core/ICalibration.hpp"
#include "owlet/core/ICalibrationStore.hpp"
#include "owlet/core/IPanTilt.hpp"
#include "owlet/hal/IAxisCalibration.hpp"

namespace owlet::core
{

/// Owns the head's calibration: asks each axis whether it accepts a change,
/// applies it, enforces "drive changes only while disabled", and persists
/// through an injected ICalibrationStore. It knows nothing about the motor
/// type: what a valid calibration is belongs to the axis.
class Calibrator final : public ICalibration
{
public:
    /// `head` is only read (to know whether the axes are powered).
    Calibrator(::owlet::hal::IAxisCalibration& pan,
               ::owlet::hal::IAxisCalibration& tilt,
               const IPanTilt& head,
               ICalibrationStore& store,
               const CalibrationRecord& factory);

    Calibrator(const Calibrator&) = delete;
    Calibrator& operator=(const Calibrator&) = delete;

    /// Apply the saved calibration if a valid one exists, else the factory one.
    /// Call at boot, before the head is enabled.
    void begin();

    // ── ICalibration ────────────────────────────────────────────────────────

    [[nodiscard]] ::owlet::hal::AxisCalibration get(Axis axis) const override;
    [[nodiscard]] Result set(Axis axis, const ::owlet::hal::AxisCalibration& cal) override;
    [[nodiscard]] Result save() override;
    [[nodiscard]] Result reset() override;
    [[nodiscard]] Source source() const override
    {
        return m_source;
    }

    /// False if the factory (build-time) record failed validation; the invalid
    /// axis then uses its own defaultCalibration().
    [[nodiscard]] bool factoryWasValid() const
    {
        return m_factoryValid;
    }

private:
    [[nodiscard]] ::owlet::hal::IAxisCalibration& axisOf(Axis axis)
    {
        return axis == Axis::Pan ? m_pan : m_tilt;
    }
    [[nodiscard]] bool enabled() const
    {
        return m_head.state() != IPanTilt::State::Disabled;
    }
    void apply(const CalibrationRecord& record);

    ::owlet::hal::IAxisCalibration& m_pan;
    ::owlet::hal::IAxisCalibration& m_tilt;
    const IPanTilt& m_head;
    ICalibrationStore& m_store;
    CalibrationRecord m_factory;
    Source m_source{Source::Factory};
    bool m_factoryValid{true};
};

} // namespace owlet::core
