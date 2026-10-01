#include "owlet/core/PanTilt.hpp"

namespace owlet::core
{

using ::owlet::Servo;

PanTilt::PanTilt(Servo& pan, Servo& tilt) : m_pan(pan), m_tilt(tilt) {}

// ── Power ───────────────────────────────────────────────────────────────────

PanTilt::Result PanTilt::enable()
{
    if (m_pan.engaged() && m_tilt.engaged())
    {
        return Result::Ok;
    }
    // Last known position, not target: after a mid-move DISABLE the horn is
    // near the position estimate, so energising there keeps the snap small.
    if (!m_pan.engageAt(m_pan.positionDeg()) || !m_tilt.engageAt(m_tilt.positionDeg()))
    {
        releaseBoth();
        return Result::Bus;
    }
    return Result::Ok;
}

PanTilt::Result PanTilt::enableAt(float panDeg, float tiltDeg)
{
    if (!inLimits(m_pan, panDeg) || !inLimits(m_tilt, tiltDeg))
    {
        return Result::Range;
    }
    if (m_pan.engaged() && m_tilt.engaged())
    {
        return move(panDeg, tiltDeg);
    }
    if (!m_pan.engageAt(panDeg) || !m_tilt.engageAt(tiltDeg))
    {
        releaseBoth();
        return Result::Bus;
    }
    return Result::Ok;
}

PanTilt::Result PanTilt::disable()
{
    const bool panOk  = m_pan.release();
    const bool tiltOk = m_tilt.release();
    return (panOk && tiltOk) ? Result::Ok : Result::Bus;
}

void PanTilt::releaseBoth()
{
    (void) m_pan.release();
    (void) m_tilt.release();
}

// ── Motion ──────────────────────────────────────────────────────────────────

PanTilt::Result PanTilt::move(float panDeg, float tiltDeg)
{
    if (state() == State::Disabled)
    {
        return Result::Disabled;
    }
    if (!inLimits(m_pan, panDeg) || !inLimits(m_tilt, tiltDeg))
    {
        return Result::Range;
    }
    m_pan.setTarget(panDeg);
    m_tilt.setTarget(tiltDeg);
    return Result::Ok;
}

PanTilt::Result PanTilt::nudge(float dPanDeg, float dTiltDeg)
{
    if (state() == State::Disabled)
    {
        return Result::Disabled;
    }
    m_pan.setTarget(m_pan.targetDeg() + dPanDeg);
    m_tilt.setTarget(m_tilt.targetDeg() + dTiltDeg);
    return Result::Ok;
}

PanTilt::Result PanTilt::home()
{
    return move(0.0f, 0.0f);
}

PanTilt::Result PanTilt::stop()
{
    if (state() == State::Disabled)
    {
        return Result::Disabled;
    }
    m_pan.stop();
    m_tilt.stop();
    return Result::Ok;
}

PanTilt::Result PanTilt::setSpeed(float degPerSec)
{
    if (degPerSec < Servo::kMinSpeedDegPerSec || degPerSec > Servo::kMaxSpeedDegPerSec)
    {
        return Result::Range;
    }
    m_pan.setSpeed(degPerSec);
    m_tilt.setSpeed(degPerSec);
    return Result::Ok;
}

bool PanTilt::tick(uint32_t dtMs)
{
    // Tick both even if one fails so a single bad axis can't freeze the other.
    const bool panOk  = m_pan.tick(dtMs);
    const bool tiltOk = m_tilt.tick(dtMs);
    return panOk && tiltOk;
}

// ── Introspection ───────────────────────────────────────────────────────────

PanTilt::State PanTilt::state() const
{
    if (!m_pan.engaged() || !m_tilt.engaged())
    {
        return State::Disabled;
    }
    return (m_pan.moving() || m_tilt.moving()) ? State::Moving : State::Holding;
}

bool PanTilt::inLimits(const Servo& s, float deg)
{
    return deg >= s.minDeg() && deg <= s.maxDeg();
}

} // namespace owlet::core
