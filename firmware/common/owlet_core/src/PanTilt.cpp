#include "owlet/core/PanTilt.hpp"

#include <cmath>

namespace owlet::core
{

using ::owlet::Servo;

PanTilt::PanTilt(Servo& pan, Servo& tilt) : m_pan(pan), m_tilt(tilt) {}

// ── Power ───────────────────────────────────────────────────────────────────

PanTilt::Result PanTilt::enable()
{
    // Last known position, not target: after a mid-move DISABLE the horn is
    // near the position estimate, so energising there keeps the snap small.
    // Axes that are still powered are left alone.
    // If the horn is outside the limits (they were narrowed, or the axis was
    // re-pulsed, while disabled), energising "where it is" would snap it to
    // the limit: the caller must choose a pose with enableAt() instead.
    if ((!m_pan.engaged() && !m_pan.positionInLimits()) ||
        (!m_tilt.engaged() && !m_tilt.positionInLimits()))
    {
        return Result::Range;
    }
    if ((!m_pan.engaged() && !m_pan.engageAt(m_pan.positionDeg())) ||
        (!m_tilt.engaged() && !m_tilt.engageAt(m_tilt.positionDeg())))
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
    // A powered axis must never snap: it slews to its pose instead.
    const auto bring = [](Servo& s, float deg)
    {
        if (s.engaged())
        {
            s.setTarget(deg);
            return true;
        }
        return s.engageAt(deg);
    };
    if (!bring(m_pan, panDeg) || !bring(m_tilt, tiltDeg))
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
    if (!bothEngaged())
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
    if (!bothEngaged())
    {
        return Result::Disabled;
    }
    if (!std::isfinite(dPanDeg) || !std::isfinite(dTiltDeg))
    {
        return Result::Range;
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
    if (!anyEngaged())
    {
        return Result::Disabled;
    }
    m_pan.stop();
    m_tilt.stop();
    return Result::Ok;
}

PanTilt::Result PanTilt::setSpeed(float degPerSec)
{
    // Negated so NaN fails the check too.
    if (!(degPerSec >= Servo::kMinSpeedDegPerSec && degPerSec <= Servo::kMaxSpeedDegPerSec))
    {
        return Result::Range;
    }
    m_pan.setSpeed(degPerSec);
    m_tilt.setSpeed(degPerSec);
    return Result::Ok;
}

PanTilt::Result PanTilt::setVelocity(float panDegPerSec, float tiltDegPerSec)
{
    if (panDegPerSec == 0.0f && tiltDegPerSec == 0.0f)
    {
        return stop();  // "stop" must work even on a partially engaged head
    }
    if (!bothEngaged())
    {
        return Result::Disabled;
    }
    const float vmax = Servo::kMaxSpeedDegPerSec;
    // Written as negated comparisons so NaN is rejected too.
    if (!(panDegPerSec >= -vmax && panDegPerSec <= vmax) ||
        !(tiltDegPerSec >= -vmax && tiltDegPerSec <= vmax))
    {
        return Result::Range;
    }
    m_pan.setVelocity(panDegPerSec);
    m_tilt.setVelocity(tiltDegPerSec);
    m_velocityAgeMs = 0;
    return Result::Ok;
}

PanTilt::Result PanTilt::setWatchdog(uint32_t ms)
{
    if (ms != 0 && (ms < kMinWatchdogMs || ms > kMaxWatchdogMs))
    {
        return Result::Range;
    }
    m_watchdogMs = ms;
    return Result::Ok;
}

bool PanTilt::inVelocityMode() const
{
    return m_pan.inVelocityMode() || m_tilt.inVelocityMode();
}

TickStatus PanTilt::tick(uint32_t dtMs)
{
    // Motion advances by at most kMaxStepMs so a starved loop doesn't lurch;
    // the watchdog below still sees the real elapsed time.
    const uint32_t step = (dtMs < kMaxStepMs) ? dtMs : kMaxStepMs;
    // Tick both even if one fails so a single bad axis can't freeze the other.
    const bool panOk  = m_pan.tick(step);
    const bool tiltOk = m_tilt.tick(step);
    TickStatus status;
    status.busOk = panOk && tiltOk;

    // The motion for this step happens first: a refreshed command is honoured
    // for exactly one watchdog period before the head is stopped.
    if (m_watchdogMs != 0 && inVelocityMode())
    {
        // Saturating add: a huge dt must trip the watchdog, never wrap past
        // it. The first test also covers a watchdog lowered below the current
        // age, and keeps the subtraction from underflowing.
        m_velocityAgeMs = (m_velocityAgeMs >= m_watchdogMs ||
                           dtMs >= m_watchdogMs - m_velocityAgeMs)
                              ? m_watchdogMs
                              : m_velocityAgeMs + dtMs;
        if (m_velocityAgeMs >= m_watchdogMs)
        {
            m_pan.stop();
            m_tilt.stop();
            status.watchdogTripped = true;
        }
    }
    return status;
}

// ── Introspection ───────────────────────────────────────────────────────────

PanTilt::State PanTilt::state() const
{
    if (!anyEngaged())
    {
        return State::Disabled;
    }
    return (m_pan.moving() || m_tilt.moving()) ? State::Moving : State::Holding;
}

PanTilt::Pose PanTilt::position() const
{
    return Pose{m_pan.positionDeg(), m_tilt.positionDeg()};
}

PanTilt::Pose PanTilt::target() const
{
    return Pose{m_pan.targetDeg(), m_tilt.targetDeg()};
}

float PanTilt::speed() const
{
    return m_pan.speed();
}

PanTilt::Limits PanTilt::limits() const
{
    return Limits{m_pan.minDeg(), m_pan.maxDeg(), m_tilt.minDeg(), m_tilt.maxDeg()};
}

PanTilt::Capabilities PanTilt::capabilities() const
{
    return Capabilities{Servo::kMinSpeedDegPerSec, Servo::kMaxSpeedDegPerSec,
                        Servo::kMaxSpeedDegPerSec, kMinWatchdogMs, kMaxWatchdogMs};
}

bool PanTilt::inLimits(const Servo& s, float deg)
{
    return deg >= s.minDeg() && deg <= s.maxDeg();  // false for NaN
}

} // namespace owlet::core
