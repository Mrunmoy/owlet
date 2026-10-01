#pragma once

#include "owlet/core/ICalibration.hpp"
#include "owlet/core/ICalibrationStore.hpp"
#include "owlet/core/IPanTilt.hpp"
#include "owlet/servo/Servo.hpp"

namespace owlet::core
{

// Owns the head's calibration: validates changes, applies them to the
// servos, enforces "pulses only while disabled", and persists through an
// injected ICalibrationStore.
class Calibrator final : public ICalibration
{
public:
    static constexpr uint16_t kMinPulseUs     = 400;
    static constexpr uint16_t kMaxPulseUs     = 2600;
    static constexpr uint16_t kMinPulseSpanUs = 200;

    // `head` is only read (to know whether the servos are powered).
    Calibrator(::owlet::Servo&          pan,
               ::owlet::Servo&          tilt,
               const IPanTilt&          head,
               ICalibrationStore&       store,
               const CalibrationRecord& factory);

    Calibrator(const Calibrator&)            = delete;
    Calibrator& operator=(const Calibrator&) = delete;

    // Apply the saved calibration if a valid one exists, else the factory one.
    // Call at boot, before the servos are enabled.
    void begin();

    // ── ICalibration ────────────────────────────────────────────────────────

    [[nodiscard]] ::owlet::Servo::Calibration get(Axis axis) const override;
    [[nodiscard]] Result set(Axis axis, const ::owlet::Servo::Calibration& cal) override;
    [[nodiscard]] Result save() override;
    [[nodiscard]] Result reset() override;
    [[nodiscard]] Source source() const override { return m_source; }

    [[nodiscard]] static bool valid(const ::owlet::Servo::Calibration& cal);
    // False if the factory (Kconfig) record failed validation; the invalid
    // axis then uses Servo::Calibration's defaults (500/1500/2500 µs, ±90°).
    [[nodiscard]] bool factoryWasValid() const { return m_factoryValid; }

private:
    [[nodiscard]] ::owlet::Servo& servo(Axis axis) { return axis == Axis::Pan ? m_pan : m_tilt; }
    [[nodiscard]] bool enabled() const { return m_head.state() != IPanTilt::State::Disabled; }
    void apply(const CalibrationRecord& record);

    ::owlet::Servo&    m_pan;
    ::owlet::Servo&    m_tilt;
    const IPanTilt&    m_head;
    ICalibrationStore& m_store;
    CalibrationRecord  m_factory;
    Source             m_source{Source::Factory};
    bool               m_factoryValid{true};
};

} // namespace owlet::core
