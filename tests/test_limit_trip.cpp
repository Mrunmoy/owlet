#include "FakeStepGenerator.hpp"
#include "RecordingEventSink.hpp"
#include "owlet/core/LimitTrip.hpp"

#include <gtest/gtest.h>

using ::owlet::core::EventId;
using ::owlet::core::LimitEnd;
using ::owlet::core::LimitTrip;

TEST(LimitTrip, CutsThePulsesAndSaysWhichSwitch)
{
    ::owlet::test::FakeStepGenerator steps;
    ::owlet::test::RecordingEventSink events;
    LimitTrip trip(steps, events, 1, LimitEnd::Max);
    ::owlet::hal::IEdgeListener& interrupt = trip;

    interrupt.onEdge();
    EXPECT_TRUE(steps.halted());
    ASSERT_EQ(events.events.size(), 1u);
    EXPECT_EQ(events.events[0].id, EventId::LimitHit);
    EXPECT_EQ(events.events[0].axis, 1);
    EXPECT_EQ(events.events[0].arg, static_cast<uint16_t>(LimitEnd::Max));
}

TEST(LimitTrip, ABouncingSwitchJustHaltsAgain)
{
    ::owlet::test::FakeStepGenerator steps;
    ::owlet::test::RecordingEventSink events;
    LimitTrip trip(steps, events, 0, LimitEnd::Min);
    for (int i = 0; i < 5; ++i)
    {
        trip.onEdge();
    }
    EXPECT_TRUE(steps.halted());
    EXPECT_EQ(steps.halts, 5) << "harmless: the bus merges the repeats into one event";
}
