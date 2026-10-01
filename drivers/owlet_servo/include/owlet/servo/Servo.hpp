#pragma once

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
// Motion is a constant-velocity slew in deg/s driven by tick(dtMs). The
// servo starts *released* (no pulse): the first pulse after power-up makes a
// hobby servo snap at full speed, so engaging is always an explicit act.
class Servo
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

    Servo(::owlet::hal::IServoChannel& channel, Calibration cal);

    Servo(const Servo&)            = delete;
    Servo& operator=(const Servo&) = delete;

    // ── Power ───────────────────────────────────────────────────────────────

    // Start pulsing at `deg` (clamped), with no slew: the horn jumps there.
    // Position and target both become `deg`.
    [[nodiscard]] bool engageAt(float deg);

    // Stop pulsing; the horn goes limp. The last commanded position is kept
    // as the best available estimate.
    [[nodiscard]] bool release();

    // ── Motion ──────────────────────────────────────────────────────────────

    // Store a slew target (clamped to soft limits). No bus traffic.
    void setTarget(float deg);

    // Freeze: target := current position.
    void stop() { m_targetDeg = m_positionDeg; }

    // Clamped to [kMinSpeedDegPerSec, kMaxSpeedDegPerSec].
    void setSpeed(float degPerSec);

    // Advance the slew by `dtMs`. Writes the channel only when the rounded
    // pulse changes. On a bus error the position estimate is left where the
    // hardware last confirmed it, and the step is retried next tick.
    [[nodiscard]] bool tick(uint32_t dtMs);

    // ── Introspection ───────────────────────────────────────────────────────

    [[nodiscard]] bool engaged() const { return m_engaged; }
    [[nodiscard]] bool moving() const { return m_engaged && m_positionDeg != m_targetDeg; }
    float positionDeg() const { return m_positionDeg; }
    float targetDeg() const { return m_targetDeg; }
    float speed() const { return m_speedDegPerSec; }
    float minDeg() const { return m_cal.minDeg; }
    float maxDeg() const { return m_cal.maxDeg; }

    uint16_t angleToPulse(float deg) const;
    float    clampDeg(float deg) const;

private:
    ::owlet::hal::IServoChannel& m_channel;
    Calibration                  m_cal;
    float                        m_positionDeg{0.0f};
    float                        m_targetDeg{0.0f};
    float                        m_speedDegPerSec{kDefaultSpeedDegPerSec};
    uint16_t                     m_lastPulseUs{0};
    bool                         m_engaged{false};
};

} // namespace owlet
