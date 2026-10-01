#include "owlet/servo/Servo.hpp"

#include <algorithm>
#include <cmath>

namespace owlet
{

Servo::Servo(::owlet::hal::IServoChannel& channel, Calibration cal)
    : m_channel(channel), m_cal(cal)
{
    m_cal.minDeg = std::max(m_cal.minDeg, -kHalfSpanDeg);
    m_cal.maxDeg = std::min(m_cal.maxDeg, kHalfSpanDeg);
    m_positionDeg = clampDeg(0.0f);
    m_targetDeg   = m_positionDeg;
}

// ── Power ───────────────────────────────────────────────────────────────────

bool Servo::engageAt(float deg)
{
    const float    clamped = clampDeg(deg);
    const uint16_t pulse   = angleToPulse(clamped);
    if (!m_channel.setPulseUs(pulse))
    {
        return false;
    }
    m_positionDeg = clamped;
    m_targetDeg   = clamped;
    m_lastPulseUs = pulse;
    m_engaged     = true;
    stop();  // a fresh engage never resumes an old velocity command
    return true;
}

bool Servo::release()
{
    // Halt motion before cutting PWM: if the idle write fails the axis stays
    // powered, and it must then hold still rather than keep slewing.
    stop();
    if (!m_channel.setIdle())
    {
        return false;
    }
    m_engaged = false;
    return true;
}

// ── Motion ──────────────────────────────────────────────────────────────────

void Servo::setTarget(float deg)
{
    m_velocityMode = false;
    m_velocity     = 0.0f;
    m_targetDeg    = clampDeg(deg);
}

void Servo::stop()
{
    m_velocityMode = false;
    m_velocity     = 0.0f;
    m_targetDeg    = m_positionDeg;
}

void Servo::setVelocity(float degPerSec)
{
    if (std::isnan(degPerSec))
    {
        stop();
        return;
    }
    const float v = std::clamp(degPerSec, -kMaxSpeedDegPerSec, kMaxSpeedDegPerSec);
    if (v == 0.0f)
    {
        stop();
        return;
    }
    m_velocityMode = true;
    m_velocity     = v;
    m_targetDeg    = m_positionDeg;
}

bool Servo::moving() const
{
    if (!m_engaged)
    {
        return false;
    }
    if (m_velocityMode)
    {
        const bool pinned = (m_velocity > 0.0f && m_positionDeg >= m_cal.maxDeg) ||
                            (m_velocity < 0.0f && m_positionDeg <= m_cal.minDeg);
        return !pinned;
    }
    return m_positionDeg != m_targetDeg;
}

void Servo::setSpeed(float degPerSec)
{
    if (std::isnan(degPerSec))
    {
        return;  // keep the last valid speed
    }
    m_speedDegPerSec = std::clamp(degPerSec, kMinSpeedDegPerSec, kMaxSpeedDegPerSec);
}

bool Servo::tick(uint32_t dtMs)
{
    if (!moving())
    {
        return true;
    }

    const float seconds = static_cast<float>(dtMs) / 1000.0f;
    float       next    = m_targetDeg;
    if (m_velocityMode)
    {
        next = clampDeg(m_positionDeg + m_velocity * seconds);
    }
    else
    {
        const float step      = m_speedDegPerSec * seconds;
        const float remaining = m_targetDeg - m_positionDeg;
        if (remaining > step)
        {
            next = m_positionDeg + step;
        }
        else if (remaining < -step)
        {
            next = m_positionDeg - step;
        }
    }

    const uint16_t pulse = angleToPulse(next);
    if (pulse != m_lastPulseUs)
    {
        if (!m_channel.setPulseUs(pulse))
        {
            return false;
        }
        m_lastPulseUs = pulse;
    }
    m_positionDeg = next;
    if (m_velocityMode)
    {
        m_targetDeg = next;  // in velocity mode the target follows the head
    }
    return true;
}

// ── Mapping ─────────────────────────────────────────────────────────────────

float Servo::clampDeg(float deg) const
{
    // std::clamp passes NaN straight through; map it to center so it can never
    // become a target or reach the float→integer pulse conversion (UB).
    return std::clamp(std::isnan(deg) ? 0.0f : deg, m_cal.minDeg, m_cal.maxDeg);
}

uint16_t Servo::angleToPulse(float deg) const
{
    const float clamped = std::clamp(std::isnan(deg) ? 0.0f : deg, -kHalfSpanDeg, kHalfSpanDeg);
    const float center  = static_cast<float>(m_cal.centerPulseUs);
    // Signed extent from center to the endpoint on the requested side. Using
    // the side's own extent is what makes the map piecewise through center
    // and lets a reversed axis fall out naturally (negative extent).
    const float end    = static_cast<float>(clamped < 0.0f ? m_cal.minPulseUs : m_cal.maxPulseUs);
    const float t      = (clamped < 0.0f ? -clamped : clamped) / kHalfSpanDeg;
    const float pulse  = center + t * (end - center);
    return static_cast<uint16_t>(pulse + 0.5f);
}

} // namespace owlet
