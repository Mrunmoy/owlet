#include "owlet/core/AxisHoming.hpp"

#include <algorithm>
#include <cmath>

namespace owlet::core
{

using ::owlet::hal::AxisCalibration;

AxisHoming::AxisHoming(::owlet::hal::IAxis& axis,
                       ::owlet::hal::IAxisCalibration& calibration,
                       const ::owlet::hal::IDigitalInput& minSwitch,
                       const ::owlet::hal::IDigitalInput* maxSwitch,
                       const Config& config)
    : m_axis(axis), m_calibration(calibration), m_minSwitch(minSwitch), m_maxSwitch(maxSwitch),
      m_cfg(config)
{
}

bool AxisHoming::start()
{
    if (busy())
    {
        return false;
    }
    m_savedSpeed = m_axis.speed();
    m_failure = Failure::None;
    m_travelDeg = 0.0f;
    m_odometerDeg = 0.0f;
    beginSeek(End::Min);
    return true;
}

void AxisHoming::abort()
{
    if (busy())
    {
        fail(Failure::Aborted);
    }
}

void AxisHoming::update()
{
    if (!busy())
    {
        return;
    }
    if (!m_axis.engaged())
    {
        fail(Failure::Aborted);
        return;
    }
    // The axis is relabelled several times on the way, so its own position
    // cannot measure distance. This sum of its movements can.
    const float position = m_axis.positionDeg();
    m_odometerDeg += position - m_lastPositionDeg;
    m_lastPositionDeg = position;

    switch (m_state)
    {
    case State::Seeking:
        if (switchAt(m_end).active())
        {
            settle(Next::BackOff);
        }
        else if (!m_axis.moving())
        {
            // Ran the whole span the soft limits allow without finding it.
            m_searchedDeg += m_axis.maxDeg() - m_axis.minDeg();
            if (m_searchedDeg >= m_cfg.searchSpanDeg)
            {
                fail(Failure::SwitchNotFound);
            }
            else
            {
                continueSeek();
            }
        }
        break;
    case State::Settling:
        if (!m_axis.moving())
        {
            if (m_next == Next::BackOff)
            {
                beginBackOff();
            }
            else
            {
                referenceFound();
            }
        }
        break;
    case State::BackingOff:
        if (!m_axis.moving())
        {
            if (switchAt(m_end).active())
            {
                fail(Failure::SwitchStuck);
            }
            else
            {
                beginApproach();
            }
        }
        break;
    case State::Approaching:
        if (switchAt(m_end).active())
        {
            settle(Next::Reference);
        }
        else if (!m_axis.moving())
        {
            fail(Failure::SwitchNotFound);
        }
        break;
    case State::Centring:
        if (!m_axis.moving())
        {
            m_state = State::Done;
        }
        break;
    case State::Idle:
    case State::Done:
    case State::Failed:
        break;
    }
}

void AxisHoming::relabel(float deg)
{
    // The axis only travels inside its soft limits, and nothing is known yet
    // about where it really is. Naming its present position is what lets it
    // be driven a full span in the wanted direction.
    if (!m_axis.engageAt(deg))
    {
        fail(Failure::Aborted);
        return;
    }
    m_lastPositionDeg = m_axis.positionDeg();
}

void AxisHoming::beginSeek(End end)
{
    m_end = end;
    m_searchedDeg = 0.0f;
    m_state = State::Seeking;
    if (switchAt(end).active())
    {
        beginBackOff(); // already on it: go straight to the slow approach
        return;
    }
    continueSeek();
}

void AxisHoming::continueSeek()
{
    const bool down = (m_end == End::Min);
    const float to = down ? m_axis.minDeg() : m_axis.maxDeg();
    relabel(down ? m_axis.maxDeg() : m_axis.minDeg());
    if (m_state == State::Failed)
    {
        return;
    }
    m_axis.setSpeed(m_cfg.seekSpeedDegPerSec);
    m_axis.setTarget(to);
    m_state = State::Seeking;
}

void AxisHoming::settle(Next next)
{
    m_axis.stop();
    m_next = next;
    m_state = State::Settling;
}

void AxisHoming::beginBackOff()
{
    const float middle = 0.5f * (m_axis.minDeg() + m_axis.maxDeg());
    const float room = 0.5f * (m_axis.maxDeg() - m_axis.minDeg());
    const float backoff = std::min(m_cfg.backoffDeg, room);
    relabel(middle);
    if (m_state == State::Failed)
    {
        return;
    }
    m_axis.setSpeed(m_cfg.seekSpeedDegPerSec);
    m_axis.setTarget(middle - towards(m_end) * backoff);
    m_state = State::BackingOff;
}

void AxisHoming::beginApproach()
{
    // Twice the back-off: the switch tripped somewhere inside it, and the
    // fast approach overshot by an unknown but small amount.
    const float room = 0.5f * (m_axis.maxDeg() - m_axis.minDeg());
    const float backoff = std::min(m_cfg.backoffDeg, room);
    m_axis.setSpeed(m_cfg.approachSpeedDegPerSec);
    m_axis.setTarget(m_axis.positionDeg() + towards(m_end) * 2.0f * backoff);
    m_state = State::Approaching;
}

void AxisHoming::referenceFound()
{
    if (m_end == End::Max)
    {
        finishWithRange(m_odometerDeg);
    }
    else if (m_maxSwitch != nullptr)
    {
        m_odometerDeg = 0.0f; // measure from here to the other switch
        beginSeek(End::Max);
    }
    else
    {
        finishWithOneSwitch();
    }
}

void AxisHoming::finishWithOneSwitch()
{
    const AxisCalibration limits = m_calibration.calibration();
    const float here = m_cfg.minSwitchDeg;
    if (here >= limits.minDeg && here <= limits.maxDeg)
    {
        relabel(here);
    }
    else
    {
        // The switch sits outside the soft limits, as it should. Widen them
        // for the moment it takes to name this position, then put them back.
        AxisCalibration wide = limits;
        wide.minDeg = std::min(limits.minDeg, here);
        wide.maxDeg = std::max(limits.maxDeg, here);
        if (!m_calibration.acceptsCalibration(wide))
        {
            fail(Failure::OutOfTravel);
            return;
        }
        m_calibration.applyCalibration(wide);
        relabel(here);
        m_calibration.applyCalibration(limits);
    }
    if (m_state != State::Failed)
    {
        centre();
    }
}

void AxisHoming::finishWithRange(float travelDeg)
{
    const float half = 0.5f * travelDeg;
    // Rounded down to the 0.1° the protocol reports limits in.
    const float inside = std::floor((half - m_cfg.marginDeg) * 10.0f) / 10.0f;

    AxisCalibration switches = m_calibration.calibration();
    switches.minDeg = -half;
    switches.maxDeg = half;
    AxisCalibration limits = switches;
    limits.minDeg = -inside;
    limits.maxDeg = inside;
    if (!(inside > 0.0f) || !m_calibration.acceptsCalibration(switches) ||
        !m_calibration.acceptsCalibration(limits))
    {
        fail(Failure::OutOfTravel);
        return;
    }
    // The axis is standing on the high switch, half the travel above zero.
    m_calibration.applyCalibration(switches);
    relabel(half);
    m_calibration.applyCalibration(limits);
    if (m_state != State::Failed)
    {
        m_travelDeg = travelDeg;
        centre();
    }
}

void AxisHoming::centre()
{
    m_axis.setSpeed(m_savedSpeed);
    m_axis.setTarget(0.0f);
    m_state = State::Centring;
}

void AxisHoming::fail(Failure failure)
{
    m_axis.stop();
    m_axis.setSpeed(m_savedSpeed);
    m_failure = failure;
    m_state = State::Failed;
}

} // namespace owlet::core
