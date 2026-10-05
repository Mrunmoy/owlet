#pragma once

#include "owlet/core/Event.hpp"
#include "owlet/core/EventNotifier.hpp"
#include "owlet/core/IEventSink.hpp"
#include "owlet/hal/IEdgeListener.hpp"
#include "owlet/hal/IStepGenerator.hpp"

#include <cstdint>

namespace owlet::core
{

/// What happens the instant a limit switch closes: the axis's step pulses
/// are cut, and the fact is posted for the main loop to deal with. Runs in
/// the switch's interrupt, so the reaction does not wait for the loop.
class LimitTrip final : public ::owlet::hal::IEdgeListener, private EventNotifier
{
public:
    /// @param steps  The pulse generator of the axis this switch guards.
    /// @param events Where LimitHit is posted.
    /// @param axis   0 = pan, 1 = tilt.
    /// @param end    Which end of travel the switch is at.
    LimitTrip(::owlet::hal::IStepGenerator& steps, IEventSink& events, uint8_t axis, LimitEnd end)
        : EventNotifier(&events), m_steps(steps), m_axis(axis), m_end(end)
    {
    }

    LimitTrip(const LimitTrip&) = delete;
    LimitTrip& operator=(const LimitTrip&) = delete;

    void onEdge() override
    {
        m_steps.halt();
        notifyLimitHit(m_axis, m_end);
    }

private:
    ::owlet::hal::IStepGenerator& m_steps;
    uint8_t m_axis;
    LimitEnd m_end;
};

} // namespace owlet::core
