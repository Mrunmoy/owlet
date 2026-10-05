#include "owlet/core/TroubleLog.hpp"

#include <limits>

namespace owlet::core
{

namespace
{

// Which events are trouble, and what they are called.
[[nodiscard]] bool codeFor(const Event& event, TroubleCode& code)
{
    switch (event.id)
    {
    case EventId::BusFault:
        code = TroubleCode::MotorBus;
        return true;
    case EventId::WatchdogExpired:
        code = TroubleCode::WatchdogExpired;
        return true;
    case EventId::DriverOffline:
        code = TroubleCode::DriverOffline;
        return true;
    case EventId::DriverFaultChanged:
        code = TroubleCode::DriverFault;
        return event.arg != 0; // a fault clearing is good news
    case EventId::LimitHit:
        code = TroubleCode::LimitHit;
        return true;
    case EventId::PositionLost:
        code = TroubleCode::PositionLost;
        return true;
    case EventId::Stalled:
        code = TroubleCode::Stalled;
        return true;
    case EventId::HomingFailed:
        code = TroubleCode::HomingFailed;
        return true;
    case EventId::EventsLost:
        code = TroubleCode::EventsLost;
        return true;
    case EventId::Arrived:
    case EventId::Homed:
    case EventId::DriverOnline:
    case EventId::BusOk:
        return false;
    }
    return false;
}

} // namespace

void TroubleLog::onEvent(const Event& event)
{
    TroubleCode code = TroubleCode::MotorBus;
    if (!codeFor(event, code))
    {
        return;
    }
    for (std::size_t i = 0; i < m_size; ++i)
    {
        TroubleRecord& r = m_records[i];
        if (r.code == code && r.axis == event.axis)
        {
            r.count = (r.count < std::numeric_limits<uint16_t>::max())
                          ? static_cast<uint16_t>(r.count + 1)
                          : r.count;
            r.detail = event.arg;
            r.lastMs = event.timeMs;
            return;
        }
    }
    if (m_size < kCapacity)
    {
        m_records[m_size++] =
            TroubleRecord{code, event.axis, 1, event.arg, event.timeMs, event.timeMs};
    }
}

} // namespace owlet::core
