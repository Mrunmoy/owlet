#pragma once

#include "owlet/core/IEventSink.hpp"
#include "owlet/hal/IAxisDiagnosticsListener.hpp"

#include <cstdint>

namespace owlet::core
{

/// Turns one motor driver's change notices into events for an axis:
/// DriverOnline / DriverOffline when it starts or stops answering,
/// DriverFaultChanged with the new fault bits, and Stalled when a stall
/// begins. Motor drivers know nothing about events; this is the join.
class AxisEventRelay final : public ::owlet::hal::IAxisDiagnosticsListener
{
public:
    /// @param axis   The axis the driver turns: 0 = pan, 1 = tilt.
    /// @param events Where the events go.
    AxisEventRelay(uint8_t axis, IEventSink& events) : m_events(events), m_axis(axis) {}

    AxisEventRelay(const AxisEventRelay&) = delete;
    AxisEventRelay& operator=(const AxisEventRelay&) = delete;

    void onDiagnosticsChanged(const ::owlet::hal::AxisDiagnostics& now) override
    {
        if (now.available != m_last.available)
        {
            (void)m_events.post(
                now.available ? EventId::DriverOnline : EventId::DriverOffline, m_axis, 0);
        }
        // A driver that has gone quiet reports no faults; that is not news.
        if (now.available && now.faults != m_last.faults)
        {
            (void)m_events.post(EventId::DriverFaultChanged, m_axis, now.faults);
        }
        if (now.stalled && !m_last.stalled)
        {
            (void)m_events.post(EventId::Stalled, m_axis, 0);
        }
        m_last = now;
    }

private:
    IEventSink& m_events;
    ::owlet::hal::AxisDiagnostics m_last{};
    uint8_t m_axis;
};

} // namespace owlet::core
