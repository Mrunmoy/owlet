#pragma once

#include "owlet/hal/IAxis.hpp"
#include "owlet/hal/IAxisCalibration.hpp"
#include "owlet/hal/IServoChannel.hpp"

#include <cstdint>

namespace owlet
{

// Logical 180° hobby servo on top of an IServoChannel.
//
// Angles are degrees from calibrated center, ±90° nominal. The angle→pulse
// map is piecewise-linear through centerPulseUs, so a servo whose electrical
// center is off-midpoint is still exact at 0°. Setting minPulseUs above
// maxPulseUs reverses the axis.
//
// Two motion modes, both driven by tick(dtMs):
//   - position: slew toward a target at speed() deg/s;
//   - velocity: turn at a signed rate until told otherwise, stopping at the
//     soft limits. Any position command leaves velocity mode.
// The
// servo starts *released* (no pulse): the first pulse after power-up makes a
// hobby servo snap at full speed, so engaging is always an explicit act.
//
// It is one implementation of hal::IAxis; the motion policy and the
// calibrator only ever see that interface.
class Servo final : public ::owlet::hal::IAxis, public ::owlet::hal::IAxisCalibration
{
public:
    struct Calibration
    {
        uint16_t minPulseUs    = 500;    // pulse at −90°
        uint16_t centerPulseUs = 1500;   // pulse at 0°
        uint16_t maxPulseUs    = 2500;   // pulse at +90°
        // Soft limits; must lie within ±90°. Targets are clamped to these.
        float    minDeg        = -90.0f;
        float    maxDeg        = +90.0f;
    };

    static constexpr float kHalfSpanDeg       = 90.0f;
    static constexpr float kMinSpeedDegPerSec = 1.0f;
    static constexpr float kMaxSpeedDegPerSec = 500.0f;
    static constexpr float kDefaultSpeedDegPerSec = 90.0f;
    // Slack for "is the horn inside its limits": one reporting step (0.1°,
    // about 1 µs), so µs rounding on a limit isn't mistaken for "outside".
    static constexpr float kPositionToleranceDeg = 0.1f;
    // Pulse widths a calibration may use, and the least span between ends.
    static constexpr uint16_t kMinPulseUs     = 400;
    static constexpr uint16_t kMaxPulseUs     = 2600;
    static constexpr uint16_t kMinPulseSpanUs = 200;

    Servo(::owlet::hal::IServoChannel& channel, Calibration cal);

    Servo(const Servo&)            = delete;
    Servo& operator=(const Servo&) = delete;

    // ── Power ───────────────────────────────────────────────────────────────

    // Start pulsing at `deg` (clamped), with no slew: the horn jumps there.
    // Position and target both become `deg`.
    [[nodiscard]] bool engageAt(float deg) override;

    // Stop pulsing; the horn goes limp. The last commanded position is kept
    // as the best available estimate.
    [[nodiscard]] bool release() override;

    // ── Motion ──────────────────────────────────────────────────────────────

    // Store a slew target (clamped to soft limits). Leaves velocity mode.
    // No bus traffic.
    void setTarget(float deg) override;

    // Freeze: target := current position. Leaves velocity mode.
    void stop() override;
    // A servo holds wherever it is told to: there is nothing sharper.
    void halt() override
    {
        stop();
    }
    [[nodiscard]] bool blocked() const override
    {
        return false;
    }

    // Velocity mode at a signed rate (clamped to ±kMaxSpeedDegPerSec).
    // Zero is the same as stop(). Independent of speed(). No bus traffic.
    void setVelocity(float degPerSec) override;

    // Replace the calibration (e.g. a reversed axis or new soft limits).
    //  - New pulses: the horn hasn't moved, so the position estimate is
    //    re-derived from the last pulse actually sent. (Changing the pulses of
    //    an *engaged* servo would make it jump; Calibrator refuses that.)
    //  - A powered axis left outside the new limits leaves velocity mode and
    //    slews back inside at speed(), never jumps.
    void setCalibration(const Calibration& cal);
    const Calibration& servoCalibration() const { return m_cal; }

    // Clamped to [kMinSpeedDegPerSec, kMaxSpeedDegPerSec].
    void setSpeed(float degPerSec) override;

    // Advance the slew by `dtMs`. Writes the channel only when the rounded
    // pulse changes. On a bus error the position estimate is left where the
    // hardware last confirmed it, and the step is retried next tick.
    [[nodiscard]] bool tick(uint32_t dtMs) override;

    // ── Introspection ───────────────────────────────────────────────────────

    [[nodiscard]] bool engaged() const override { return m_engaged; }
    [[nodiscard]] bool moving() const override;
    [[nodiscard]] bool inVelocityMode() const override { return m_velocityMode; }
    float velocity() const { return m_velocity; }
    [[nodiscard]] float positionDeg() const override { return m_positionDeg; }
    [[nodiscard]] float targetDeg() const override { return m_targetDeg; }
    [[nodiscard]] float speed() const override { return m_speedDegPerSec; }
    [[nodiscard]] float minDeg() const override { return m_cal.minDeg; }
    [[nodiscard]] float maxDeg() const override { return m_cal.maxDeg; }
    [[nodiscard]] Capabilities capabilities() const override
    {
        return Capabilities{kMinSpeedDegPerSec, kMaxSpeedDegPerSec, kMaxSpeedDegPerSec};
    }

    uint16_t angleToPulse(float deg) const;
    float    pulseToAngle(uint16_t pulseUs) const;  // inverse of angleToPulse
    float    clampDeg(float deg) const;
    // False if the horn may be outside the limits, or its angle is unknown
    // (its last pulse lies beyond the current pulse range).
    [[nodiscard]] bool positionInLimits() const override
    {
        return m_positionKnown && m_positionDeg >= m_cal.minDeg - kPositionToleranceDeg &&
               m_positionDeg <= m_cal.maxDeg + kPositionToleranceDeg;
    }
    [[nodiscard]] bool positionKnown() const { return m_positionKnown; }

    // ── IAxisCalibration ────────────────────────────────────────────────────
    // The drive numbers are the pulse widths (µs) at −90°, 0° and +90°.

    [[nodiscard]] ::owlet::hal::AxisCalibration calibration() const override
    {
        return toAxisCalibration(m_cal);
    }
    [[nodiscard]] ::owlet::hal::AxisCalibration defaultCalibration() const override
    {
        return toAxisCalibration(Calibration{});
    }
    [[nodiscard]] bool acceptsCalibration(const ::owlet::hal::AxisCalibration& cal) const override;
    // New pulses would make a powered horn jump; new limits alone do not.
    [[nodiscard]] bool
    calibrationNeedsRelease(const ::owlet::hal::AxisCalibration& cal) const override;
    void applyCalibration(const ::owlet::hal::AxisCalibration& cal) override
    {
        setCalibration(fromAxisCalibration(cal));
    }

    [[nodiscard]] static constexpr ::owlet::hal::AxisCalibration
    toAxisCalibration(const Calibration& c)
    {
        return ::owlet::hal::AxisCalibration{c.minPulseUs, c.centerPulseUs, c.maxPulseUs, c.minDeg,
                                             c.maxDeg};
    }
    [[nodiscard]] static constexpr Calibration
    fromAxisCalibration(const ::owlet::hal::AxisCalibration& a)
    {
        return Calibration{a.driveMin, a.driveCenter, a.driveMax, a.minDeg, a.maxDeg};
    }

private:
    ::owlet::hal::IServoChannel& m_channel;
    Calibration                  m_cal;
    float                        m_positionDeg{0.0f};
    float                        m_targetDeg{0.0f};
    float                        m_speedDegPerSec{kDefaultSpeedDegPerSec};
    float                        m_velocity{0.0f};
    uint16_t                     m_lastPulseUs{0};
    bool                         m_engaged{false};
    bool                         m_velocityMode{false};
    bool                         m_positionKnown{true};
};

} // namespace owlet
