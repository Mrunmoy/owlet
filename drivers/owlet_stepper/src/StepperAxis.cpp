#include "owlet/stepper/StepperAxis.hpp"

#include <algorithm>
#include <cmath>

namespace owlet
{

using ::owlet::hal::AxisCalibration;

StepperAxis::StepperAxis(::owlet::hal::IStepGenerator& steps,
                         ::owlet::hal::IDigitalOutput& enable,
                         const Config& config)
    : StepperAxis(steps, enable, config, LimitSwitches{})
{
}

StepperAxis::StepperAxis(::owlet::hal::IStepGenerator& steps,
                         ::owlet::hal::IDigitalOutput& enable,
                         const Config& config,
                         LimitSwitches limits)
    : m_steps(steps), m_enable(enable), m_cfg(config), m_limits(limits)
{
    // A nonsensical build-time value must not become a division by zero or a
    // motor that can never start.
    if (!(m_cfg.stepsPerDeg > 0.0f) || !std::isfinite(m_cfg.stepsPerDeg))
    {
        m_cfg.stepsPerDeg = 1.0f;
    }
    if (!(m_cfg.accelDegPerSec2 > 0.0f) || !std::isfinite(m_cfg.accelDegPerSec2))
    {
        m_cfg.accelDegPerSec2 = 1.0f;
    }
    if (!(m_cfg.maxSpeedDegPerSec >= kMinSpeedDegPerSec) || !std::isfinite(m_cfg.maxSpeedDegPerSec))
    {
        m_cfg.maxSpeedDegPerSec = kMinSpeedDegPerSec;
    }
    // NaN compares false, so a non-finite travel also lands on the cap.
    if (!(m_cfg.travelMinDeg >= -kMaxTravelDeg))
    {
        m_cfg.travelMinDeg = -kMaxTravelDeg;
    }
    if (!(m_cfg.travelMaxDeg <= kMaxTravelDeg))
    {
        m_cfg.travelMaxDeg = kMaxTravelDeg;
    }
    if (!acceptsCalibration(AxisCalibration{0, 0, 0, m_cfg.minDeg, m_cfg.maxDeg}))
    {
        m_cfg.travelMinDeg = std::min(m_cfg.travelMinDeg, -1.0f);
        m_cfg.travelMaxDeg = std::max(m_cfg.travelMaxDeg, 1.0f);
        m_cfg.minDeg = m_cfg.travelMinDeg;
        m_cfg.maxDeg = m_cfg.travelMaxDeg;
    }
    m_speedDegPerSec = std::min(kDefaultSpeedDegPerSec, m_cfg.maxSpeedDegPerSec);
    setLimits(m_cfg.minDeg, m_cfg.maxDeg);
}

// ── Power ───────────────────────────────────────────────────────────────────

bool StepperAxis::engageAt(float deg)
{
    m_positionSteps = toSteps(clampDeg(deg));
    m_targetSteps = m_positionSteps;
    m_rate = 0.0f;
    m_fraction = 0.0f;
    m_refusedMs = 0;
    m_blocked = false;
    m_velocityMode = false;
    m_velocityDegPerSec = 0.0f;
    m_enable.set(true);
    m_engaged = true;
    return true;
}

bool StepperAxis::release()
{
    // No more steps from this instant. The step count stays as the best
    // estimate, although an unpowered shaft can be turned by hand.
    m_rate = 0.0f;
    m_fraction = 0.0f;
    m_velocityMode = false;
    m_velocityDegPerSec = 0.0f;
    m_targetSteps = m_positionSteps;
    m_enable.set(false);
    m_engaged = false;
    return true;
}

// ── Motion ──────────────────────────────────────────────────────────────────

void StepperAxis::setTarget(float deg)
{
    m_velocityMode = false;
    m_velocityDegPerSec = 0.0f;
    m_targetSteps = toSteps(clampDeg(deg));
}

void StepperAxis::stop()
{
    m_velocityMode = false;
    m_velocityDegPerSec = 0.0f;
    // Rest where braking from the present rate ends: v² / 2a further on.
    const float accel = m_cfg.accelDegPerSec2 * m_cfg.stepsPerDeg;
    const float distance = (m_rate * m_rate) / (2.0f * accel);
    const float ahead = (m_rate >= 0.0f) ? distance : -distance;
    const float resting = static_cast<float>(m_positionSteps) + ahead;
    // An axis already outside its limits rests where it is rather than being
    // pulled to the limit by a stop.
    const int32_t lo = std::min(m_minSteps, m_positionSteps);
    const int32_t hi = std::max(m_maxSteps, m_positionSteps);
    m_targetSteps = std::clamp(static_cast<int32_t>(std::lround(resting)), lo, hi);
}

void StepperAxis::halt()
{
    m_rate = 0.0f;
    m_fraction = 0.0f;
    m_velocityMode = false;
    m_velocityDegPerSec = 0.0f;
    m_targetSteps = m_positionSteps;
}

void StepperAxis::setVelocity(float degPerSec)
{
    if (std::isnan(degPerSec))
    {
        stop();
        return;
    }
    const float v = std::clamp(degPerSec, -m_cfg.maxSpeedDegPerSec, m_cfg.maxSpeedDegPerSec);
    if (v == 0.0f)
    {
        stop();
        return;
    }
    m_velocityMode = true;
    m_velocityDegPerSec = v;
}

void StepperAxis::setSpeed(float degPerSec)
{
    if (std::isnan(degPerSec))
    {
        return; // keep the last valid speed
    }
    m_speedDegPerSec = std::clamp(degPerSec, kMinSpeedDegPerSec, m_cfg.maxSpeedDegPerSec);
}

bool StepperAxis::pinnedAtLimit() const
{
    return (m_velocityDegPerSec > 0.0f && m_positionSteps >= m_maxSteps) ||
           (m_velocityDegPerSec < 0.0f && m_positionSteps <= m_minSteps);
}

bool StepperAxis::moving() const
{
    if (!m_engaged)
    {
        return false;
    }
    if (m_rate != 0.0f)
    {
        return true;
    }
    return m_velocityMode ? !pinnedAtLimit() : m_positionSteps != m_targetSteps;
}

bool StepperAxis::tick(uint32_t dtMs)
{
    if (!m_engaged || dtMs == 0)
    {
        return true;
    }
    // Something outside (a limit switch's interrupt) stopped the pulses in
    // mid-burst. The move is over and the count is no longer exact; make the
    // generator usable again so the axis can be driven away.
    if (m_steps.halted())
    {
        halt();
        m_blocked = true;
        m_steps.resume();
        return true;
    }
    dtMs = std::min(dtMs, kMaxTickMs);
    const float seconds = static_cast<float>(dtMs) / 1000.0f;
    const float accel = m_cfg.accelDegPerSec2 * m_cfg.stepsPerDeg; // steps/s²

    // Where this mode is heading, and how fast it may cruise there. Velocity
    // mode heads for the soft limit in its direction of travel, so reaching a
    // limit is the same ramp-down as reaching a target.
    int32_t goal = m_targetSteps;
    float cruise = m_speedDegPerSec * m_cfg.stepsPerDeg;
    if (m_velocityMode)
    {
        goal = pinnedAtLimit() ? m_positionSteps
                               : (m_velocityDegPerSec > 0.0f ? m_maxSteps : m_minSteps);
        cruise = std::fabs(m_velocityDegPerSec) * m_cfg.stepsPerDeg;
    }

    const int32_t remaining = goal - m_positionSteps;
    const float distance = static_cast<float>(remaining);
    // The fastest rate from which the axis can still brake onto the goal.
    const float stoppable = std::sqrt(2.0f * accel * std::fabs(distance));
    const float wanted =
        (remaining == 0) ? 0.0f : std::copysign(std::min(cruise, stoppable), distance);
    const float change = accel * seconds;
    m_rate = std::clamp(wanted, m_rate - change, m_rate + change);

    const float exact = m_rate * seconds + m_fraction;
    int32_t steps = static_cast<int32_t>(exact); // toward zero
    m_fraction = exact - static_cast<float>(steps);
    // Heading for the goal, never step past it.
    if ((steps > 0 && remaining >= 0 && steps > remaining) ||
        (steps < 0 && remaining <= 0 && steps < remaining))
    {
        steps = remaining;
        m_fraction = 0.0f;
    }

    // A pressed switch outranks everything above: the soft limits may be
    // wrong, or the position may be. Nothing goes toward it, and there is
    // no ramp, because every further step is a step into the stop.
    const ::owlet::hal::IDigitalInput* ahead =
        (steps > 0) ? m_limits.max : ((steps < 0) ? m_limits.min : nullptr);
    if (ahead != nullptr && ahead->active())
    {
        halt();
        m_blocked = true;
        return true;
    }

    if (steps != 0)
    {
        if (!m_steps.emit(m_cfg.reversed ? -steps : steps, dtMs * 1000u))
        {
            // Nothing went out. Carry at most one step of the demand, so the
            // retry is immediate (a refusal that keeps coinciding with the
            // final step can't starve it) but no backlog builds into a burst.
            m_fraction = std::clamp(exact, -1.0f, 1.0f);
            m_refusedMs = std::min(m_refusedMs + dtMs, kStallReportMs);
            return m_refusedMs < kStallReportMs;
        }
        m_refusedMs = 0;
        m_blocked = false;
        m_positionSteps += steps;
    }
    if (m_positionSteps == goal)
    {
        m_rate = 0.0f; // arrived: the sqrt profile got here at a crawl
        m_fraction = 0.0f;
    }
    return true;
}

// ── Calibration ─────────────────────────────────────────────────────────────

bool StepperAxis::acceptsCalibration(const AxisCalibration& c) const
{
    if (c.driveMin != 0 || c.driveCenter != 0 || c.driveMax != 0)
    {
        return false;
    }
    // Negated comparisons so NaN fails as well.
    return std::isfinite(c.minDeg) && std::isfinite(c.maxDeg) && c.minDeg >= m_cfg.travelMinDeg &&
           c.minDeg <= 0.0f && c.maxDeg >= 0.0f && c.maxDeg <= m_cfg.travelMaxDeg &&
           c.minDeg < c.maxDeg;
}

void StepperAxis::applyCalibration(const AxisCalibration& cal)
{
    setLimits(cal.minDeg, cal.maxDeg);
    if (m_engaged && !positionInLimits())
    {
        // Travel back inside at speed(); never jump.
        m_velocityMode = false;
        m_velocityDegPerSec = 0.0f;
        m_targetSteps = std::clamp(m_positionSteps, m_minSteps, m_maxSteps);
    }
    else if (!m_velocityMode)
    {
        m_targetSteps = std::clamp(m_targetSteps, m_minSteps, m_maxSteps);
    }
}

// ── Mapping ─────────────────────────────────────────────────────────────────

void StepperAxis::setLimits(float minDeg, float maxDeg)
{
    m_minDeg = minDeg;
    m_maxDeg = maxDeg;
    m_minSteps = toSteps(minDeg);
    m_maxSteps = toSteps(maxDeg);
}

int32_t StepperAxis::toSteps(float deg) const
{
    return static_cast<int32_t>(std::lround(deg * m_cfg.stepsPerDeg));
}

float StepperAxis::clampDeg(float deg) const
{
    // std::clamp passes NaN straight through; map it to centre so it can
    // never become a target or reach the float→integer step conversion (UB).
    return std::clamp(std::isnan(deg) ? 0.0f : deg, m_minDeg, m_maxDeg);
}

} // namespace owlet
