#include "FakeClock.hpp"
#include "owlet/core/EventBus.hpp"

#include <gtest/gtest.h>

#include <vector>

using ::owlet::core::Event;
using ::owlet::core::EventBus;
using ::owlet::core::EventId;
using ::owlet::core::IEventObserver;
using ::owlet::core::LimitEnd;
using ::owlet::core::StaticEventBus;
using ::owlet::test::FakeClock;

namespace
{

// Counts lock and unlock, and fails a test that leaves them unbalanced.
class CountingLock final : public ::owlet::hal::IInterruptLock
{
public:
    [[nodiscard]] uint32_t lock() override
    {
        ++depth;
        ++locks;
        return 0xA5;
    }
    void unlock(uint32_t saved) override
    {
        EXPECT_EQ(saved, 0xA5u) << "unlock must be given what lock returned";
        --depth;
    }

    int depth{0};
    int locks{0};
};

class Recorder final : public IEventObserver
{
public:
    void onEvent(const Event& event) override
    {
        seen.push_back(event);
    }

    std::vector<Event> seen;
};

Event make(EventId id, uint8_t axis = Event::kHead, uint16_t arg = 0, uint32_t timeMs = 0)
{
    Event e;
    e.id = id;
    e.axis = axis;
    e.arg = arg;
    e.timeMs = timeMs;
    return e;
}

// Posts `e` as if at the time it carries.
bool post(EventBus& bus, FakeClock& clock, const Event& e)
{
    clock.t = e.timeMs;
    return bus.post(e.id, e.axis, e.arg);
}

std::vector<EventId> ids(const std::vector<Event>& events)
{
    std::vector<EventId> out;
    for (const Event& e : events)
    {
        out.push_back(e.id);
    }
    return out;
}

} // namespace

TEST(EventBus, DeliversInOrderToEveryObserver)
{
    CountingLock lock;
    FakeClock clock;
    Recorder first;
    Recorder second;
    StaticEventBus<8, 3> bus(lock, clock, {&first, nullptr, &second}); // a null entry is skipped

    EXPECT_FALSE(bus.pending());
    EXPECT_TRUE(post(bus, clock, make(EventId::Arrived, Event::kHead, 0, 100)));
    EXPECT_TRUE(post(bus, clock, make(EventId::WatchdogExpired, Event::kHead, 0, 120)));
    EXPECT_TRUE(bus.pending());
    EXPECT_TRUE(first.seen.empty()) << "nothing is delivered until the loop dispatches";

    EXPECT_EQ(bus.dispatch(), 2u);
    EXPECT_EQ(ids(first.seen), (std::vector<EventId>{EventId::Arrived, EventId::WatchdogExpired}));
    EXPECT_EQ(first.seen, second.seen);
    EXPECT_EQ(first.seen[0].timeMs, 100u);
    EXPECT_FALSE(bus.pending());
    EXPECT_EQ(bus.dispatch(), 0u);
    EXPECT_EQ(lock.depth, 0);
    EXPECT_GT(lock.locks, 0) << "the queue is guarded";
}

TEST(EventBus, AnEventPostedByAnObserverIsDeliveredAfterNotInside)
{
    class Chained final : public IEventObserver
    {
    public:
        void onEvent(const Event& event) override
        {
            EXPECT_EQ(depth, 0) << "handlers never nest";
            ++depth;
            seen.push_back(event.id);
            if (event.id == EventId::LimitHit)
            {
                bus->post(EventId::PositionLost, event.axis, 0);
            }
            --depth;
        }
        EventBus* bus{nullptr};
        std::vector<EventId> seen;
        int depth{0};
    };
    CountingLock lock;
    FakeClock clock;
    Chained policy;
    Recorder wire;
    StaticEventBus<8, 2> bus(lock, clock, {&policy, &wire});
    policy.bus = &bus;

    post(bus, clock, make(EventId::LimitHit, 0, static_cast<uint16_t>(LimitEnd::Max)));
    post(bus, clock, make(EventId::Arrived));
    EXPECT_EQ(bus.dispatch(), 3u);
    // The wire sees the limit, then what was already waiting, then the consequence.
    EXPECT_EQ(ids(wire.seen),
              (std::vector<EventId>{EventId::LimitHit, EventId::Arrived, EventId::PositionLost}));
}

TEST(EventBus, AFullQueueDropsCountsAndSaysSo)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<3, 1> bus(lock, clock, {&wire});

    EXPECT_TRUE(post(bus, clock, make(EventId::Arrived, Event::kHead, 0, 10)));
    EXPECT_TRUE(post(bus, clock, make(EventId::DriverOnline, 0, 0, 11)));
    EXPECT_TRUE(post(bus, clock, make(EventId::DriverOnline, 1, 0, 12)));
    EXPECT_FALSE(post(bus, clock, make(EventId::DriverOffline, 0, 0, 13)));
    EXPECT_FALSE(post(bus, clock, make(EventId::DriverOffline, 1, 0, 14)));
    EXPECT_EQ(bus.stats().dropped, 2u);
    EXPECT_EQ(bus.stats().highWater, 3);

    bus.dispatch();
    ASSERT_EQ(wire.seen.size(), 4u);
    EXPECT_EQ(wire.seen.back().id, EventId::EventsLost);
    EXPECT_EQ(wire.seen.back().arg, 2) << "the notice says how many";
    EXPECT_EQ(wire.seen.back().timeMs, 14u) << "and when the last one was lost";

    wire.seen.clear();
    EXPECT_TRUE(post(bus, clock, make(EventId::Arrived)));
    bus.dispatch();
    EXPECT_EQ(ids(wire.seen), std::vector<EventId>{EventId::Arrived})
        << "the loss is reported once";
}

TEST(EventBus, ALimitHitGetsThroughAFullQueue)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<2, 1> bus(lock, clock, {&wire});
    post(bus, clock, make(EventId::Arrived));
    post(bus, clock, make(EventId::Arrived));
    ASSERT_FALSE(post(bus, clock, make(EventId::Arrived))) << "the ring is full";

    EXPECT_TRUE(
        post(bus, clock, make(EventId::LimitHit, 1, static_cast<uint16_t>(LimitEnd::Min), 77)));
    EXPECT_TRUE(post(bus, clock, make(EventId::Stalled, 0, 0, 78)));
    EXPECT_EQ(bus.stats().dropped, 1u) << "only the ordinary event was lost";

    bus.dispatch();
    ASSERT_GE(wire.seen.size(), 2u);
    // Latched events come first, in a fixed order: by axis, limits before stall.
    EXPECT_EQ(wire.seen[0], make(EventId::Stalled, 0, 0, 78));
    EXPECT_EQ(wire.seen[1], make(EventId::LimitHit, 1, static_cast<uint16_t>(LimitEnd::Min), 77));
    EXPECT_EQ(wire.seen.back().id, EventId::EventsLost);
}

TEST(EventBus, ALatchRaisedManyTimesIsOneEvent)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<4, 1> bus(lock, clock, {&wire});
    for (uint32_t t = 0; t < 50; ++t) // a bouncing switch
    {
        EXPECT_TRUE(post(
            bus, clock, make(EventId::LimitHit, 0, static_cast<uint16_t>(LimitEnd::Max), 500 + t)));
    }
    bus.dispatch();
    ASSERT_EQ(wire.seen.size(), 1u);
    EXPECT_EQ(wire.seen[0].timeMs, 500u) << "stamped with the first edge";

    post(bus, clock, make(EventId::LimitHit, 0, static_cast<uint16_t>(LimitEnd::Max), 900));
    bus.dispatch();
    EXPECT_EQ(wire.seen.size(), 2u) << "and it can be raised again once delivered";
}

TEST(EventBus, BothEndsAndEveryAxisHaveTheirOwnLatch)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<2, 1> bus(lock, clock, {&wire});
    post(bus, clock, make(EventId::LimitHit, 0, static_cast<uint16_t>(LimitEnd::Min)));
    post(bus, clock, make(EventId::LimitHit, 0, static_cast<uint16_t>(LimitEnd::Max)));
    post(bus, clock, make(EventId::LimitHit, 3, static_cast<uint16_t>(LimitEnd::Max)));
    post(bus, clock, make(EventId::Stalled, 3));
    EXPECT_EQ(bus.dispatch(), 4u);

    wire.seen.clear();
    EXPECT_FALSE(post(bus, clock, make(EventId::LimitHit, EventBus::kMaxAxes, 0)))
        << "no latch for that axis";
    EXPECT_FALSE(post(bus, clock, make(EventId::LimitHit, 0, 2)))
        << "nor for an end that does not exist";
    EXPECT_EQ(bus.stats().dropped, 2u);
}

TEST(EventBus, RepeatedFaultChangesForAnAxisMerge)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<4, 1> bus(lock, clock, {&wire});
    post(bus, clock, make(EventId::DriverFaultChanged, 0, 0x01, 10));
    post(bus, clock, make(EventId::DriverFaultChanged, 1, 0x04, 11));
    post(bus, clock, make(EventId::DriverFaultChanged, 0, 0x03, 12)); // pan again: newer state
    post(bus, clock, make(EventId::DriverFaultChanged, 0, 0x02, 13));
    EXPECT_EQ(bus.stats().highWater, 2) << "one slot per axis, however often it changes";

    bus.dispatch();
    ASSERT_EQ(wire.seen.size(), 2u);
    EXPECT_EQ(wire.seen[0], make(EventId::DriverFaultChanged, 0, 0x02, 13))
        << "the latest, in the first one's place";
    EXPECT_EQ(wire.seen[1], make(EventId::DriverFaultChanged, 1, 0x04, 11));
}

TEST(EventBus, AnObserverThatAlwaysRepliesCannotHangDispatch)
{
    class Echo final : public IEventObserver
    {
    public:
        void onEvent(const Event& event) override
        {
            ++calls;
            bus->post(event.id, event.axis, event.arg);
        }
        EventBus* bus{nullptr};
        int calls{0};
    };
    CountingLock lock;
    FakeClock clock;
    Echo echo;
    StaticEventBus<4, 1> bus(lock, clock, {&echo});
    echo.bus = &bus;
    post(bus, clock, make(EventId::Arrived));
    const std::size_t delivered = bus.dispatch();
    EXPECT_LE(delivered, 16u) << "one call does a bounded amount of work";
    EXPECT_EQ(static_cast<std::size_t>(echo.calls), delivered);
    EXPECT_TRUE(bus.pending()) << "the rest waits for the next pass";
}

TEST(EventBus, WrapsAroundItsStorage)
{
    CountingLock lock;
    FakeClock clock;
    Recorder wire;
    StaticEventBus<3, 1> bus(lock, clock, {&wire});
    for (uint32_t i = 0; i < 20; ++i)
    {
        ASSERT_TRUE(
            post(bus, clock, make(EventId::Arrived, Event::kHead, static_cast<uint16_t>(i), i)));
        ASSERT_TRUE(
            post(bus, clock, make(EventId::BusOk, Event::kHead, static_cast<uint16_t>(i), i)));
        ASSERT_EQ(bus.dispatch(), 2u);
    }
    ASSERT_EQ(wire.seen.size(), 40u);
    EXPECT_EQ(wire.seen[38], make(EventId::Arrived, Event::kHead, 19, 19));
    EXPECT_EQ(wire.seen[39], make(EventId::BusOk, Event::kHead, 19, 19));
    EXPECT_EQ(bus.stats().posted, 40u);
    EXPECT_EQ(bus.stats().dropped, 0u);
}

TEST(EventBus, ListenersCanBeAttachedAfterConstruction)
{
    CountingLock lock;
    FakeClock clock;
    Recorder first;
    Recorder second;
    Recorder third;
    StaticEventBus<4, 2> bus(lock, clock);
    EXPECT_TRUE(bus.attach(first));
    EXPECT_TRUE(bus.attach(second));
    EXPECT_FALSE(bus.attach(third)) << "every place is taken";

    clock.t = 42;
    bus.post(EventId::Arrived, Event::kHead, 0);
    bus.dispatch();
    EXPECT_EQ(first.seen, (std::vector<Event>{make(EventId::Arrived, Event::kHead, 0, 42)}));
    EXPECT_EQ(second.seen, first.seen);
    EXPECT_TRUE(third.seen.empty());
}
