#pragma once

#include "owlet/core/Event.hpp"
#include "owlet/core/IEventObserver.hpp"

#include <cstdint>

namespace owlet::core
{

/// A listener that reacts to events by name. Derive from it and override
/// only the reactions you care about; the rest do nothing.
///
/// @code
///   class StatusLight final : public EventDelegate
///   {
///       void onArrived() override { m_led.set(false); }
///       void onLimitHit(uint8_t axis, LimitEnd end) override { m_led.blink(); }
///   };
/// @endcode
///
/// Attach it to the head's EventBus. Every reaction runs in the main loop,
/// one at a time, never from an interrupt.
class EventDelegate : public IEventObserver
{
public:
    /// Routes an event to its named reaction.
    void onEvent(const Event& event) final
    {
        m_eventTimeMs = event.timeMs;
        switch (event.id)
        {
        case EventId::Arrived:
            onArrived();
            break;
        case EventId::WatchdogExpired:
            onWatchdogExpired();
            break;
        case EventId::BusFault:
            onBusFault();
            break;
        case EventId::BusOk:
            onBusOk();
            break;
        case EventId::LimitHit:
            onLimitHit(event.axis, static_cast<LimitEnd>(event.arg));
            break;
        case EventId::Homed:
            onHomed();
            break;
        case EventId::HomingFailed:
            onHomingFailed(event.axis, event.arg);
            break;
        case EventId::PositionLost:
            onPositionLost(event.axis);
            break;
        case EventId::Stalled:
            onStalled(event.axis);
            break;
        case EventId::DriverFaultChanged:
            onDriverFaultChanged(event.axis, event.arg);
            break;
        case EventId::DriverOnline:
            onDriverOnline(event.axis);
            break;
        case EventId::DriverOffline:
            onDriverOffline(event.axis);
            break;
        case EventId::EventsLost:
            onEventsLost(event.arg);
            break;
        }
    }

protected:
    // ── Motion ──────────────────────────────────────────────────────────────

    /// A position move reached its target.
    virtual void onArrived() {}
    /// Velocity mode was stopped for lack of a command.
    virtual void onWatchdogExpired() {}

    // ── Knowing where the head is ───────────────────────────────────────────

    /// A limit switch closed. Axes are numbered 0 = pan, 1 = tilt, here and below.
    virtual void onLimitHit(uint8_t /*axis*/, LimitEnd /*end*/) {}
    /// The step count of an axis can no longer be trusted.
    virtual void onPositionLost(uint8_t /*axis*/) {}
    /// A motor stopped following its steps.
    virtual void onStalled(uint8_t /*axis*/) {}
    /// Homing finished; the position can be trusted again.
    virtual void onHomed() {}
    /// Homing gave up on an axis. The second argument is the AxisHoming::Failure.
    virtual void onHomingFailed(uint8_t /*axis*/, uint16_t /*reason*/) {}

    // ── Motor drivers ───────────────────────────────────────────────────────

    /// A motor driver started answering.
    virtual void onDriverOnline(uint8_t /*axis*/) {}
    /// A motor driver stopped answering.
    virtual void onDriverOffline(uint8_t /*axis*/) {}
    /// A motor driver's fault bits changed. The second argument is the new set; 0 = cleared.
    virtual void onDriverFaultChanged(uint8_t /*axis*/, uint16_t /*faults*/) {}

    // ── The system itself ───────────────────────────────────────────────────

    /// A motor could not be commanded.
    virtual void onBusFault() {}
    /// Commands are getting through again.
    virtual void onBusOk() {}
    /// Events were dropped before they could be delivered.
    virtual void onEventsLost(uint16_t /*count*/) {}

    /// @return When the event now being handled was raised, on the device clock.
    [[nodiscard]] uint32_t eventTimeMs() const
    {
        return m_eventTimeMs;
    }

private:
    uint32_t m_eventTimeMs{0};
};

} // namespace owlet::core
