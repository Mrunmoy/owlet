#include "RecordingEventSink.hpp"
#include "owlet/core/AxisEventRelay.hpp"

#include <gtest/gtest.h>

#include <vector>

using ::owlet::core::AxisEventRelay;
using ::owlet::core::Event;
using ::owlet::core::EventId;
using ::owlet::hal::AxisDiagnostics;
using ::owlet::test::RecordingEventSink;

namespace
{

AxisDiagnostics state(bool available, uint16_t faults = 0, bool stalled = false)
{
    AxisDiagnostics d;
    d.available = available;
    d.faults = faults;
    d.stalled = stalled;
    return d;
}

} // namespace

TEST(AxisEventRelay, ADriverComingAndGoingIsAnnouncedForItsAxis)
{
    RecordingEventSink sink;
    AxisEventRelay relay(1, sink);
    relay.onDiagnosticsChanged(state(true));
    relay.onDiagnosticsChanged(state(false));
    ASSERT_EQ(sink.events.size(), 2u);
    EXPECT_EQ(sink.events[0].id, EventId::DriverOnline);
    EXPECT_EQ(sink.events[0].axis, 1);
    EXPECT_EQ(sink.events[1].id, EventId::DriverOffline);
}

TEST(AxisEventRelay, FaultChangesCarryTheNewBits)
{
    RecordingEventSink sink;
    AxisEventRelay relay(0, sink);
    relay.onDiagnosticsChanged(state(true));
    (void)sink.take();
    sink.events.clear();

    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTempWarning));
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTempWarning)); // no change
    relay.onDiagnosticsChanged(state(true, 0));                                 // cleared
    ASSERT_EQ(sink.events.size(), 2u);
    EXPECT_EQ(sink.events[0].id, EventId::DriverFaultChanged);
    EXPECT_EQ(sink.events[0].arg, AxisDiagnostics::kOverTempWarning);
    EXPECT_EQ(sink.events[1].arg, 0) << "a fault clearing is a change too";
}

TEST(AxisEventRelay, ADriverAlreadyFaultyWhenItAppearsSaysSo)
{
    RecordingEventSink sink;
    AxisEventRelay relay(0, sink);
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kShortA));
    EXPECT_EQ(sink.take(),
              (std::vector<EventId>{EventId::DriverOnline, EventId::DriverFaultChanged}));
}

TEST(AxisEventRelay, GoingOfflineIsNotAlsoAFaultChange)
{
    RecordingEventSink sink;
    AxisEventRelay relay(0, sink);
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTemp));
    (void)sink.take();
    relay.onDiagnosticsChanged(state(false));
    EXPECT_EQ(sink.take(), std::vector<EventId>{EventId::DriverOffline});
}

TEST(AxisEventRelay, AStallIsAnnouncedWhenItBeginsNotWhileItLasts)
{
    RecordingEventSink sink;
    AxisEventRelay relay(0, sink);
    relay.onDiagnosticsChanged(state(true));
    (void)sink.take();
    relay.onDiagnosticsChanged(state(true, 0, true));
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTempWarning, true));
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTempWarning, false));
    relay.onDiagnosticsChanged(state(true, AxisDiagnostics::kOverTempWarning, true));
    EXPECT_EQ(
        sink.take(),
        (std::vector<EventId>{EventId::Stalled, EventId::DriverFaultChanged, EventId::Stalled}));
}
