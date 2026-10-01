#include "owlet/core/Calibrator.hpp"

#include <algorithm>
#include <cmath>

namespace owlet::core
{

using ::owlet::Servo;

Calibrator::Calibrator(Servo&                   pan,
                       Servo&                   tilt,
                       const IPanTilt&          head,
                       ICalibrationStore&       store,
                       const CalibrationRecord& factory)
    : m_pan(pan), m_tilt(tilt), m_head(head), m_store(store), m_factory(factory)
{
    // Kconfig ranges are per field, so a bad combination can slip through; the
    // device must never run with a mapping it would itself refuse.
    for (Servo::Calibration* c : {&m_factory.pan, &m_factory.tilt})
    {
        if (!valid(*c))
        {
            *c             = Servo::Calibration{};
            m_factoryValid = false;
        }
    }
}

void Calibrator::begin()
{
    CalibrationRecord saved{};
    if (m_store.load(saved) && valid(saved.pan) && valid(saved.tilt))
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
    m_pan.setCalibration(record.pan);
    m_tilt.setCalibration(record.tilt);
}

Servo::Calibration Calibrator::get(Axis axis) const
{
    return axis == Axis::Pan ? m_pan.calibration() : m_tilt.calibration();
}

Calibrator::Result Calibrator::set(Axis axis, const Servo::Calibration& in)
{
    // Limits are kept at the 0.1° resolution $CAL reports, so a client's
    // read-modify-write round trip is exact.
    Servo::Calibration cal = in;
    cal.minDeg             = std::round(cal.minDeg * 10.0f) / 10.0f;
    cal.maxDeg             = std::round(cal.maxDeg * 10.0f) / 10.0f;
    if (!valid(cal))
    {
        return Result::Range;
    }
    const Servo::Calibration current = get(axis);
    const bool pulsesChange = cal.minPulseUs != current.minPulseUs ||
                              cal.centerPulseUs != current.centerPulseUs ||
                              cal.maxPulseUs != current.maxPulseUs;
    // A powered servo would jump to wherever its current pulse means under the
    // new mapping; limits alone only constrain future targets.
    if (pulsesChange && enabled())
    {
        return Result::Enabled;
    }
    servo(axis).setCalibration(cal);
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

bool Calibrator::valid(const Servo::Calibration& c)
{
    const auto inPulseRange = [](uint16_t us) { return us >= kMinPulseUs && us <= kMaxPulseUs; };
    if (!inPulseRange(c.minPulseUs) || !inPulseRange(c.centerPulseUs) ||
        !inPulseRange(c.maxPulseUs))
    {
        return false;
    }
    const uint16_t lo = std::min(c.minPulseUs, c.maxPulseUs);
    const uint16_t hi = std::max(c.minPulseUs, c.maxPulseUs);
    if (hi - lo < kMinPulseSpanUs || !(lo < c.centerPulseUs && c.centerPulseUs < hi))
    {
        return false;
    }
    // Negated comparisons so NaN fails as well.
    return std::isfinite(c.minDeg) && std::isfinite(c.maxDeg) && c.minDeg >= -90.0f &&
           c.minDeg <= 0.0f && c.maxDeg >= 0.0f && c.maxDeg <= 90.0f && c.minDeg < c.maxDeg;
}

} // namespace owlet::core
