#pragma once

#include "owlet/core/Event.hpp"
#include "owlet/core/IEventObserver.hpp"
#include "owlet/core/IEventSink.hpp"
#include "owlet/hal/IClock.hpp"
#include "owlet/hal/IInterruptLock.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::core
{

/// Carries events from whatever noticed something to whoever reacts to it.
///
/// Sources post(), from the main loop or from an interrupt. The main loop
/// calls dispatch(), which hands each event to every observer in the order
/// they were listed. Observers are named when the bus is built or attached
/// during start-up, and do not change after that.
///
/// **What a full queue does.** A limit hit or a stall must be reported even
/// if nothing else is, so those are not queued at all: each sets a latch that
/// dispatch() turns into one event. Raising a latch that is already set
/// changes nothing. Everything else waits in a fixed ring; when the ring is
/// full the event is dropped and counted, and the listeners are sent an
/// EventsLost carrying the count. A DriverFaultChanged for an axis that
/// already has one waiting replaces it.
///
/// **Order of delivery.** Latched events first, then queued events oldest
/// first, then EventsLost. Events posted by an observer during dispatch()
/// are delivered in the same call, after the one being handled.
///
/// The storage and the observer list belong to the caller; StaticEventBus
/// supplies both.
class EventBus : public IEventSink
{
public:
    /// The most axes whose limit and stall latches are kept.
    static constexpr uint8_t kMaxAxes = 4;

    /// Counters for judging whether the queue is big enough.
    struct Stats
    {
        uint32_t posted{0};    ///< Events accepted, latched ones included.
        uint32_t dropped{0};   ///< Events refused because the ring was full.
        uint16_t highWater{0}; ///< Most events ever waiting in the ring.
    };

    /// @param lock          Guards the queue against interrupts.
    /// @param clock         Stamps each event as it is posted.
    /// @param storage       Room for `capacity` events.
    /// @param observers     `observerCount` listeners in delivery order. Null
    ///                      entries are free places for attach().
    EventBus(::owlet::hal::IInterruptLock& lock,
             const ::owlet::hal::IClock& clock,
             Event* storage,
             std::size_t capacity,
             IEventObserver** observers,
             std::size_t observerCount);

    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;

    bool post(EventId id, uint8_t axis, uint16_t arg) override;

    /// Adds a listener after those already named. For start-up, where a
    /// listener often cannot exist before the bus it needs. Main loop only.
    /// @return false if every place is taken.
    [[nodiscard]] bool attach(IEventObserver& observer);

    /// Delivers everything that is waiting. Main loop only.
    ///
    /// Bounded: an observer that posts an event every time it receives one
    /// cannot keep this from returning.
    /// @return The number of events delivered.
    std::size_t dispatch();

    /// @return true if dispatch() would deliver something.
    [[nodiscard]] bool pending() const;

    [[nodiscard]] Stats stats() const;

private:
    [[nodiscard]] static bool isLatched(EventId id)
    {
        return id == EventId::LimitHit || id == EventId::Stalled;
    }

    bool raise(const Event& event);
    bool enqueue(const Event& event);
    [[nodiscard]] bool takeLatched(Event& out);
    [[nodiscard]] bool takeQueued(Event& out);
    [[nodiscard]] bool takeLost(Event& out);
    void deliver(const Event& event);

    // Latch slots: two limit ends and one stall per axis.
    static constexpr std::size_t kLatchesPerAxis = 3;
    static constexpr std::size_t kLatchCount = kMaxAxes * kLatchesPerAxis;

    ::owlet::hal::IInterruptLock& m_lock;
    const ::owlet::hal::IClock& m_clock;
    Event* m_storage;
    std::size_t m_capacity;
    IEventObserver** m_observers;
    std::size_t m_observerCount;
    std::size_t m_head{0};  ///< Next event to deliver.
    std::size_t m_count{0}; ///< Events waiting in the ring.
    uint32_t m_latched{0};  ///< One bit per latch slot.
    std::array<uint32_t, kLatchCount> m_latchTimeMs{};
    uint32_t m_lastTimeMs{0}; ///< Stamps the EventsLost notice.
    uint16_t m_lostUnreported{0};
    Stats m_stats{};
};

namespace detail
{

/// The arrays a StaticEventBus lends to its EventBus. A base class, listed
/// first, so that they exist before the bus is given pointers to them.
template <std::size_t Capacity, std::size_t Observers>
struct EventBusStorage
{
    EventBusStorage() = default;
    explicit EventBusStorage(std::array<IEventObserver*, Observers> observers)
        : listeners(observers)
    {
    }

    std::array<Event, Capacity> ring{};
    std::array<IEventObserver*, Observers> listeners{};
};

} // namespace detail

/// An EventBus together with its storage, sized at compile time.
///
/// @tparam Capacity  Events the ring can hold.
/// @tparam Observers Places for listeners.
template <std::size_t Capacity, std::size_t Observers>
class StaticEventBus final : private detail::EventBusStorage<Capacity, Observers>, public EventBus
{
    using Storage = detail::EventBusStorage<Capacity, Observers>;

public:
    static_assert(Capacity > 0, "the ring needs room for at least one event");

    /// A bus whose listeners will all be attached later.
    StaticEventBus(::owlet::hal::IInterruptLock& lock, const ::owlet::hal::IClock& clock)
        : Storage(),
          EventBus(
              lock, clock, Storage::ring.data(), Capacity, Storage::listeners.data(), Observers)
    {
    }

    StaticEventBus(::owlet::hal::IInterruptLock& lock,
                   const ::owlet::hal::IClock& clock,
                   std::array<IEventObserver*, Observers> observers)
        : Storage(observers),
          EventBus(
              lock, clock, Storage::ring.data(), Capacity, Storage::listeners.data(), Observers)
    {
    }
};

} // namespace owlet::core
