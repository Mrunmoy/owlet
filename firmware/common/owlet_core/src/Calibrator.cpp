#include "owlet/core/Calibrator.hpp"

#include <cmath>

namespace owlet::core
{

using ::owlet::hal::AxisCalibration;
using ::owlet::hal::IAxisCalibration;

Calibrator::Calibrator(IAxisCalibration& pan,
                       IAxisCalibration& tilt,
                       const IPanTilt& head,
                       ICalibrationStore& store,
                       const CalibrationRecord& factory)
    : m_pan(pan), m_tilt(tilt), m_head(head), m_store(store), m_factory(factory)
{
    // Build-time ranges are per field, so a bad combination can slip through;
    // the device must never run with a mapping it would itself refuse.
    if (!m_pan.acceptsCalibration(m_factory.pan))
    {
        m_factory.pan = m_pan.defaultCalibration();
        m_factoryValid = false;
    }
    if (!m_tilt.acceptsCalibration(m_factory.tilt))
    {
        m_factory.tilt = m_tilt.defaultCalibration();
        m_factoryValid = false;
    }
}

void Calibrator::begin()
{
    CalibrationRecord saved{};
    if (m_store.load(saved) && m_pan.acceptsCalibration(saved.pan) &&
        m_tilt.acceptsCalibration(saved.tilt))
    {
        apply(saved);
        m_source = Source::Saved;
        return;
    }
    apply(m_factory);
    m_source = Source::Factory;
}

void Calibrator::apply(const CalibrationRecord& record)
{
    m_pan.applyCalibration(record.pan);
    m_tilt.applyCalibration(record.tilt);
}

AxisCalibration Calibrator::get(Axis axis) const
{
    return axis == Axis::Pan ? m_pan.calibration() : m_tilt.calibration();
}

Calibrator::Result Calibrator::set(Axis axis, const AxisCalibration& in)
{
    // Limits are kept at the 0.1° resolution $CAL reports, so a client's
    // read-modify-write round trip is exact.
    AxisCalibration cal = in;
    cal.minDeg = std::round(cal.minDeg * 10.0f) / 10.0f;
    cal.maxDeg = std::round(cal.maxDeg * 10.0f) / 10.0f;
    IAxisCalibration& target = axisOf(axis);
    if (!target.acceptsCalibration(cal))
    {
        return Result::Range;
    }
    // A powered axis must not move by itself because its mapping changed;
    // limits alone only constrain future targets.
    if (target.calibrationNeedsRelease(cal) && enabled())
    {
        return Result::Enabled;
    }
    target.applyCalibration(cal);
    m_source = Source::Modified;
    return Result::Ok;
}

Calibrator::Result Calibrator::save()
{
    if (!m_store.save(CalibrationRecord{get(Axis::Pan), get(Axis::Tilt)}))
    {
        return Result::Storage;
    }
    m_source = Source::Saved;
    return Result::Ok;
}

Calibrator::Result Calibrator::reset()
{
    if (enabled())
    {
        return Result::Enabled;
    }
    // Erase first: if that fails, nothing changes, so RAM, flash and the
    // reported source keep agreeing.
    if (!m_store.erase())
    {
        return Result::Storage;
    }
    apply(m_factory);
    m_source = Source::Factory;
    return Result::Ok;
}

} // namespace owlet::core
