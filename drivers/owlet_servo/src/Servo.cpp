#include "owlet/servo/Servo.hpp"

#include <algorithm>

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
    return true;
}

bool Servo::release()
{
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
    m_targetDeg = clampDeg(deg);
}

void Servo::setSpeed(float degPerSec)
{
    m_speedDegPerSec = std::clamp(degPerSec, kMinSpeedDegPerSec, kMaxSpeedDegPerSec);
}

bool Servo::tick(uint32_t dtMs)
{
    if (!moving())
    {
        return true;
    }

    const float step      = m_speedDegPerSec * static_cast<float>(dtMs) / 1000.0f;
    const float remaining = m_targetDeg - m_positionDeg;
    float       next      = m_targetDeg;
    if (remaining > step)
    {
        next = m_positionDeg + step;
    }
    else if (remaining < -step)
    {
        next = m_positionDeg - step;
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
    return true;
}

// ── Mapping ─────────────────────────────────────────────────────────────────

float Servo::clampDeg(float deg) const
{
    return std::clamp(deg, m_cal.minDeg, m_cal.maxDeg);
}

uint16_t Servo::angleToPulse(float deg) const
{
    const float clamped = std::clamp(deg, -kHalfSpanDeg, kHalfSpanDeg);
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
