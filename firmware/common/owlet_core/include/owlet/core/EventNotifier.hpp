#pragma once

#include "owlet/core/Event.hpp"
#include "owlet/core/IEventSink.hpp"

#include <cstdint>

namespace owlet::core
{

/// The announcing side of the head's events, by name. Whatever notices a
/// change derives from this (privately) and says so in one word:
///
/// @code
///   if (m_arrivalOwed && state() == State::Holding)
///   {
///       notifyArrived();
///   }
/// @endcode
///
/// With no sink the calls do nothing, so a class can announce without
/// caring whether anyone is listening. Every call is safe from an interrupt.
class EventNotifier
{
public:
    explicit EventNotifier(IEventSink* sink = nullptr) : m_sink(sink) {}

    // ── Motion ──────────────────────────────────────────────────────────────

    void notifyArrived() const
    {
        post(EventId::Arrived);
    }
    void notifyWatchdogExpired() const
    {
        post(EventId::WatchdogExpired);
    }

    // ── Knowing where the head is ───────────────────────────────────────────

    void notifyLimitHit(uint8_t axis, LimitEnd end) const
    {
        post(EventId::LimitHit, axis, static_cast<uint16_t>(end));
    }
    void notifyPositionLost(uint8_t axis) const
    {
        post(EventId::PositionLost, axis);
    }
    void notifyStalled(uint8_t axis) const
    {
        post(EventId::Stalled, axis);
    }
    void notifyHomed() const
    {
        post(EventId::Homed);
    }
    void notifyHomingFailed(uint8_t axis, uint16_t reason) const
    {
        post(EventId::HomingFailed, axis, reason);
    }

    // ── Motor drivers ───────────────────────────────────────────────────────

    void notifyDriverOnline(uint8_t axis) const
    {
        post(EventId::DriverOnline, axis);
    }
    void notifyDriverOffline(uint8_t axis) const
    {
        post(EventId::DriverOffline, axis);
    }
    void notifyDriverFaultChanged(uint8_t axis, uint16_t faults) const
    {
        post(EventId::DriverFaultChanged, axis, faults);
    }

    // ── The system itself ───────────────────────────────────────────────────

    void notifyBusFault() const
    {
        post(EventId::BusFault);
    }
    void notifyBusOk() const
    {
        post(EventId::BusOk);
    }

private:
    void post(EventId id, uint8_t axis = Event::kHead, uint16_t arg = 0) const
    {
        if (m_sink != nullptr)
        {
            (void)m_sink->post(id, axis, arg);
        }
    }

    IEventSink* m_sink;
};

} // namespace owlet::core
