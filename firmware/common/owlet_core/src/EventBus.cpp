#include "owlet/core/EventBus.hpp"

#include <limits>

namespace owlet::core
{

using ::owlet::hal::InterruptGuard;

namespace
{

constexpr std::size_t kStallSlot = 2; // after the two limit ends

} // namespace

EventBus::EventBus(::owlet::hal::IInterruptLock& lock,
                   const ::owlet::hal::IClock& clock,
                   Event* storage,
                   std::size_t capacity,
                   IEventObserver** observers,
                   std::size_t observerCount)
    : m_lock(lock), m_clock(clock), m_storage(storage), m_capacity(capacity),
      m_observers(observers), m_observerCount(observerCount)
{
}

bool EventBus::post(EventId id, uint8_t axis, uint16_t arg)
{
    Event event;
    event.id = id;
    event.axis = axis;
    event.arg = arg;
    event.timeMs = m_clock.nowMs(); // read before interrupts are masked
    return isLatched(id) ? raise(event) : enqueue(event);
}

bool EventBus::attach(IEventObserver& observer)
{
    for (std::size_t i = 0; i < m_observerCount; ++i)
    {
        if (m_observers[i] == nullptr)
        {
            m_observers[i] = &observer;
            return true;
        }
    }
    return false;
}

bool EventBus::raise(const Event& event)
{
    const bool isLimit = event.id == EventId::LimitHit;
    const bool valid = event.axis < kMaxAxes && (!isLimit || event.arg <= 1);
    const std::size_t slot =
        valid ? event.axis * kLatchesPerAxis + (isLimit ? event.arg : kStallSlot) : 0;
    const uint32_t bit = uint32_t{1} << slot;

    const InterruptGuard guard(m_lock);
    if (!valid)
    {
        ++m_stats.dropped;
        m_lostUnreported = (m_lostUnreported < std::numeric_limits<uint16_t>::max())
                               ? static_cast<uint16_t>(m_lostUnreported + 1)
                               : m_lostUnreported;
        m_lastTimeMs = event.timeMs;
        return false;
    }
    if ((m_latched & bit) == 0)
    {
        m_latched |= bit;
        m_latchTimeMs[slot] = event.timeMs; // the first edge is the one that counts
        ++m_stats.posted;
    }
    return true;
}

bool EventBus::enqueue(const Event& event)
{
    const InterruptGuard guard(m_lock);
    if (event.id == EventId::DriverFaultChanged)
    {
        // Only the latest fault state of an axis matters to a listener.
        for (std::size_t i = 0; i < m_count; ++i)
        {
            Event& waiting = m_storage[(m_head + i) % m_capacity];
            if (waiting.id == event.id && waiting.axis == event.axis)
            {
                waiting.arg = event.arg;
                waiting.timeMs = event.timeMs;
                ++m_stats.posted;
                return true;
            }
        }
    }
    if (m_count == m_capacity)
    {
        ++m_stats.dropped;
        m_lostUnreported = (m_lostUnreported < std::numeric_limits<uint16_t>::max())
                               ? static_cast<uint16_t>(m_lostUnreported + 1)
                               : m_lostUnreported;
        m_lastTimeMs = event.timeMs;
        return false;
    }
    m_storage[(m_head + m_count) % m_capacity] = event;
    ++m_count;
    ++m_stats.posted;
    if (m_count > m_stats.highWater)
    {
        m_stats.highWater = static_cast<uint16_t>(m_count);
    }
    return true;
}

bool EventBus::takeLatched(Event& out)
{
    const InterruptGuard guard(m_lock);
    for (std::size_t slot = 0; slot < kLatchCount; ++slot)
    {
        const uint32_t bit = uint32_t{1} << slot;
        if ((m_latched & bit) == 0)
        {
            continue;
        }
        m_latched &= ~bit;
        const std::size_t kind = slot % kLatchesPerAxis;
        out.id = (kind == kStallSlot) ? EventId::Stalled : EventId::LimitHit;
        out.axis = static_cast<uint8_t>(slot / kLatchesPerAxis);
        out.arg = (kind == kStallSlot) ? uint16_t{0} : static_cast<uint16_t>(kind);
        out.timeMs = m_latchTimeMs[slot];
        return true;
    }
    return false;
}

bool EventBus::takeQueued(Event& out)
{
    const InterruptGuard guard(m_lock);
    if (m_count == 0)
    {
        return false;
    }
    out = m_storage[m_head];
    m_head = (m_head + 1) % m_capacity;
    --m_count;
    return true;
}

bool EventBus::takeLost(Event& out)
{
    const InterruptGuard guard(m_lock);
    if (m_lostUnreported == 0)
    {
        return false;
    }
    out.id = EventId::EventsLost;
    out.axis = Event::kHead;
    out.arg = m_lostUnreported;
    out.timeMs = m_lastTimeMs;
    m_lostUnreported = 0;
    return true;
}

void EventBus::deliver(const Event& event)
{
    for (std::size_t i = 0; i < m_observerCount; ++i)
    {
        if (m_observers[i] != nullptr)
        {
            m_observers[i]->onEvent(event);
        }
    }
}

std::size_t EventBus::dispatch()
{
    // Each loop has a fixed budget, so observers that post from their
    // handlers add work to this call only up to a point; the rest waits.
    std::size_t delivered = 0;
    Event event;
    for (std::size_t i = 0; i < kLatchCount && takeLatched(event); ++i)
    {
        deliver(event);
        ++delivered;
    }
    for (std::size_t i = 0; i < m_capacity && takeQueued(event); ++i)
    {
        deliver(event);
        ++delivered;
    }
    if (takeLost(event))
    {
        deliver(event);
        ++delivered;
    }
    return delivered;
}

bool EventBus::pending() const
{
    const InterruptGuard guard(m_lock);
    return m_latched != 0 || m_count != 0 || m_lostUnreported != 0;
}

EventBus::Stats EventBus::stats() const
{
    const InterruptGuard guard(m_lock);
    return m_stats;
}

} // namespace owlet::core
