#include "owlet/core/TroubleLog.hpp"

#include <gtest/gtest.h>

using ::owlet::core::Event;
using ::owlet::core::EventId;
using ::owlet::core::TroubleCode;
using ::owlet::core::TroubleLog;

namespace
{

Event make(EventId id, uint8_t axis = Event::kHead, uint16_t arg = 0, uint32_t timeMs = 0)
{
    Event e;
    e.id = id;
    e.axis = axis;
    e.arg = arg;
    e.timeMs = timeMs;
    return e;
}

} // namespace

TEST(TroubleLog, StartsEmpty)
{
    const TroubleLog log;
    EXPECT_EQ(log.size(), 0u);
}

TEST(TroubleLog, RecordsWhatWentWrongWhereAndWhen)
{
    TroubleLog log;
    log.onEvent(make(EventId::DriverFaultChanged, 1, 0x06, 1500));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log.at(0).code, TroubleCode::DriverFault);
    EXPECT_EQ(log.at(0).axis, 1);
    EXPECT_EQ(log.at(0).count, 1);
    EXPECT_EQ(log.at(0).detail, 0x06);
    EXPECT_EQ(log.at(0).firstMs, 1500u);
    EXPECT_EQ(log.at(0).lastMs, 1500u);
}

TEST(TroubleLog, ARepeatRaisesTheCountAndKeepsTheFirstTime)
{
    TroubleLog log;
    log.onEvent(make(EventId::WatchdogExpired, Event::kHead, 0, 100));
    log.onEvent(make(EventId::WatchdogExpired, Event::kHead, 0, 900));
    log.onEvent(make(EventId::WatchdogExpired, Event::kHead, 0, 4000));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log.at(0).count, 3);
    EXPECT_EQ(log.at(0).firstMs, 100u);
    EXPECT_EQ(log.at(0).lastMs, 4000u);
}

TEST(TroubleLog, TheSameTroubleOnAnotherAxisIsItsOwnRecord)
{
    TroubleLog log;
    log.onEvent(make(EventId::LimitHit, 0, 1));
    log.onEvent(make(EventId::LimitHit, 1, 0));
    log.onEvent(make(EventId::PositionLost, 0));
    ASSERT_EQ(log.size(), 3u);
    EXPECT_EQ(log.at(0).axis, 0);
    EXPECT_EQ(log.at(1).axis, 1);
    EXPECT_EQ(log.at(2).code, TroubleCode::PositionLost);
}

TEST(TroubleLog, GoodNewsIsNotTrouble)
{
    TroubleLog log;
    log.onEvent(make(EventId::Arrived));
    log.onEvent(make(EventId::Homed));
    log.onEvent(make(EventId::DriverOnline, 0));
    log.onEvent(make(EventId::BusOk));
    log.onEvent(make(EventId::DriverFaultChanged, 0, 0)); // a fault clearing
    EXPECT_EQ(log.size(), 0u);
}

TEST(TroubleLog, EveryKindOfTroubleHasItsCode)
{
    TroubleLog log;
    log.onEvent(make(EventId::BusFault));
    log.onEvent(make(EventId::WatchdogExpired));
    log.onEvent(make(EventId::DriverOffline, 0));
    log.onEvent(make(EventId::DriverFaultChanged, 0, 2));
    log.onEvent(make(EventId::LimitHit, 0, 1));
    log.onEvent(make(EventId::PositionLost, 0));
    log.onEvent(make(EventId::Stalled, 0));
    log.onEvent(make(EventId::HomingFailed, 0, 3));
    log.onEvent(make(EventId::EventsLost, Event::kHead, 4));
    ASSERT_EQ(log.size(), 9u);
    const int expected[] = {10, 11, 20, 21, 30, 31, 32, 40, 50};
    for (std::size_t i = 0; i < 9; ++i)
    {
        EXPECT_EQ(static_cast<int>(log.at(i).code), expected[i]);
    }
}

TEST(TroubleLog, TheCountStopsAtItsLimit)
{
    TroubleLog log;
    for (int i = 0; i < 70000; ++i)
    {
        log.onEvent(make(EventId::Stalled, 0));
    }
    EXPECT_EQ(log.at(0).count, 65535);
}

TEST(TroubleLog, ClearForgetsEverything)
{
    TroubleLog log;
    log.onEvent(make(EventId::Stalled, 0));
    log.clear();
    EXPECT_EQ(log.size(), 0u);
    log.onEvent(make(EventId::Stalled, 1, 0, 77));
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log.at(0).firstMs, 77u);
}

TEST(TroubleLog, AFullTableKeepsWhatItHasAndStillCountsRepeats)
{
    TroubleLog log;
    for (uint8_t axis = 0; axis < 30; ++axis) // far more axes than a head has
    {
        log.onEvent(make(EventId::Stalled, axis));
    }
    EXPECT_EQ(log.size(), TroubleLog::kCapacity);
    log.onEvent(make(EventId::Stalled, 0));
    EXPECT_EQ(log.at(0).count, 2);
}
