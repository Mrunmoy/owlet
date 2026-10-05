#include "FakeAxis.hpp"
#include "FakeServoChannel.hpp"
#include "RecordingEventSink.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/servo/Servo.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using ::owlet::Servo;
using ::owlet::core::PanTilt;
using ::owlet::test::FakeServoChannel;
using Result = ::owlet::core::PanTilt::Result;
using State  = ::owlet::core::PanTilt::State;

namespace
{

struct Rig
{
    FakeServoChannel panCh;
    FakeServoChannel tiltCh;
    Servo            pan{panCh, Servo::Calibration{}};
    Servo            tilt{tiltCh, Servo::Calibration{}};
    PanTilt          head{pan, tilt};
};

} // namespace

TEST(PanTilt, BootsDisabledAndLimp)
{
    Rig r;
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_TRUE(r.panCh.writes.empty());
    EXPECT_TRUE(r.tiltCh.writes.empty());
}

TEST(PanTilt, MotionCommandsRequireEnable)
{
    Rig r;
    EXPECT_EQ(r.head.move(10.0f, 10.0f), Result::Disabled);
    EXPECT_EQ(r.head.nudge(1.0f, 1.0f), Result::Disabled);
    EXPECT_EQ(r.head.home(), Result::Disabled);
    EXPECT_EQ(r.head.stop(), Result::Disabled);
    EXPECT_TRUE(r.panCh.writes.empty());
}

TEST(PanTilt, EnableEnergisesAtLastKnownPosition)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_EQ(r.panCh.lastPulseUs, 1500);
    EXPECT_EQ(r.tiltCh.lastPulseUs, 1500);
}

TEST(PanTilt, EnableAtPoseSnapsBothAxes)
{
    Rig r;
    ASSERT_EQ(r.head.enableAt(45.0f, -45.0f), Result::Ok);
    EXPECT_EQ(r.panCh.lastPulseUs, 2000);
    EXPECT_EQ(r.tiltCh.lastPulseUs, 1000);
    EXPECT_FLOAT_EQ(r.head.position().pan, 45.0f);
}

TEST(PanTilt, EnableAtOutOfRangeIsRejectedWithoutMotion)
{
    Rig r;
    EXPECT_EQ(r.head.enableAt(95.0f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_TRUE(r.panCh.writes.empty());
}

TEST(PanTilt, EnableAtWhileEnabledSlewsInsteadOfSnapping)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    const std::size_t writes = r.panCh.writes.size();
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), Result::Ok);
    EXPECT_EQ(r.panCh.writes.size(), writes);  // no immediate jump
    EXPECT_EQ(r.head.state(), State::Moving);
    EXPECT_FLOAT_EQ(r.head.target().pan, 60.0f);
}

TEST(PanTilt, ReEnableAfterMidMoveDisableUsesPositionNotTarget)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    ASSERT_TRUE(r.head.tick(100).busOk);  // pan at 20°
    ASSERT_EQ(r.head.disable(), Result::Ok);
    EXPECT_TRUE(r.panCh.idle);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_NEAR(r.head.position().pan, 20.0f, 1e-3f);
    EXPECT_EQ(r.head.state(), State::Holding);
}

TEST(PanTilt, MoveRejectsOutOfLimitTargets)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.move(90.5f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.move(0.0f, -91.0f), Result::Range);
    EXPECT_EQ(r.head.move(90.0f, -90.0f), Result::Ok);
}

TEST(PanTilt, MoveSlewsBothAxesIndependently)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(30.0f, -10.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    EXPECT_NEAR(r.head.position().pan, 10.0f, 1e-3f);
    EXPECT_NEAR(r.head.position().tilt, -10.0f, 1e-3f);  // tilt already arrived
    EXPECT_EQ(r.head.state(), State::Moving);
    ASSERT_TRUE(r.head.tick(100).busOk);
    ASSERT_TRUE(r.head.tick(100).busOk);
    EXPECT_EQ(r.head.state(), State::Holding);
}

TEST(PanTilt, NudgeIsRelativeToTargetAndClamps)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    ASSERT_EQ(r.head.nudge(5.0f, -3.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 85.0f);
    EXPECT_FLOAT_EQ(r.head.target().tilt, -3.0f);
    ASSERT_EQ(r.head.nudge(50.0f, 0.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 90.0f);
}

TEST(PanTilt, HomeTargetsCenter)
{
    Rig r;
    ASSERT_EQ(r.head.enableAt(40.0f, 40.0f), Result::Ok);
    ASSERT_EQ(r.head.home(), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 0.0f);
    EXPECT_FLOAT_EQ(r.head.target().tilt, 0.0f);
}

TEST(PanTilt, StopFreezesWhereItIs)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(50.0f, 50.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    ASSERT_TRUE(r.head.tick(50).busOk);
    ASSERT_EQ(r.head.stop(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_NEAR(r.head.target().pan, 15.0f, 1e-3f);
}

TEST(PanTilt, SpeedRangeIsValidated)
{
    Rig r;
    EXPECT_EQ(r.head.setSpeed(0.5f), Result::Range);
    EXPECT_EQ(r.head.setSpeed(501.0f), Result::Range);
    EXPECT_EQ(r.head.setSpeed(250.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.speed(), 250.0f);
    EXPECT_FLOAT_EQ(r.pan.speed(), 250.0f);
    EXPECT_FLOAT_EQ(r.tilt.speed(), 250.0f);
}

TEST(PanTilt, EnableBusFailureLeavesBothAxesLimp)
{
    Rig r;
    r.tiltCh.failNext = true;
    EXPECT_EQ(r.head.enable(), Result::Bus);
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_TRUE(r.panCh.idle);
}

TEST(PanTilt, ReportsLimitsFromServos)
{
    FakeServoChannel   a;
    FakeServoChannel   b;
    Servo::Calibration panCal;
    panCal.minDeg = -60.0f;
    panCal.maxDeg = 70.0f;
    Servo::Calibration tiltCal;
    tiltCal.minDeg = -30.0f;
    Servo   pan{a, panCal};
    Servo   tilt{b, tiltCal};
    PanTilt head{pan, tilt};
    const PanTilt::Limits lim = head.limits();
    EXPECT_FLOAT_EQ(lim.panMin, -60.0f);
    EXPECT_FLOAT_EQ(lim.panMax, 70.0f);
    EXPECT_FLOAT_EQ(lim.tiltMin, -30.0f);
    EXPECT_FLOAT_EQ(lim.tiltMax, 90.0f);
}

TEST(PanTilt, TickReportsBusFault)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(10.0f, 0.0f), Result::Ok);
    r.panCh.failNext = true;
    EXPECT_FALSE(r.head.tick(20).busOk);
    EXPECT_TRUE(r.head.tick(20).busOk);
}

TEST(PanTilt, UsableThroughInterfaces)
{
    Rig                         r;
    ::owlet::core::IPanTilt&    control = r.head;
    ::owlet::core::ITickable&   motion  = r.head;
    ASSERT_EQ(control.enable(), Result::Ok);
    ASSERT_EQ(control.move(5.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(motion.tick(1000).busOk);
    EXPECT_FLOAT_EQ(control.position().pan, 5.0f);
}

// ── Velocity mode + watchdog ────────────────────────────────────────────────

TEST(PanTiltVelocity, RequiresEnable)
{
    Rig r;
    EXPECT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Disabled);
}

TEST(PanTiltVelocity, RangeIsValidated)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.setVelocity(500.5f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.setVelocity(0.0f, -501.0f), Result::Range);
    EXPECT_EQ(r.head.setVelocity(500.0f, -500.0f), Result::Ok);
}

TEST(PanTiltVelocity, MovesBothAxes)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(20.0f, -10.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    EXPECT_NEAR(r.head.position().pan, 2.0f, 1e-4f);
    EXPECT_NEAR(r.head.position().tilt, -1.0f, 1e-4f);
    EXPECT_EQ(r.head.state(), State::Moving);
}

TEST(PanTiltVelocity, WatchdogDefaultsTo500ms)
{
    Rig r;
    EXPECT_EQ(r.head.watchdogMs(), 500u);
}

TEST(PanTiltVelocity, WatchdogStopsStaleVelocityOnce)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 10.0f), Result::Ok);
    EXPECT_FALSE(r.head.tick(200).watchdogTripped);
    EXPECT_FALSE(r.head.tick(200).watchdogTripped);
    EXPECT_TRUE(r.head.tick(100).watchdogTripped);  // 500 ms without a refresh
    EXPECT_EQ(r.head.state(), State::Holding);      // stopped, still enabled
    const float here = r.head.position().pan;
    EXPECT_FALSE(r.head.tick(1000).watchdogTripped);  // reported once
    EXPECT_FLOAT_EQ(r.head.position().pan, here);
}

TEST(PanTiltVelocity, RefreshResetsWatchdog)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    for (int i = 0; i < 10; ++i)
    {
        ASSERT_EQ(r.head.setVelocity(5.0f, 0.0f), Result::Ok);
        EXPECT_FALSE(r.head.tick(400).watchdogTripped);
    }
    EXPECT_EQ(r.head.state(), State::Moving);
}

TEST(PanTiltVelocity, WatchdogCanBeDisabled)
{
    Rig r;
    ASSERT_EQ(r.head.setWatchdog(0), Result::Ok);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(1.0f, 0.0f), Result::Ok);
    for (int i = 0; i < 50; ++i)
    {
        EXPECT_FALSE(r.head.tick(100).watchdogTripped);
    }
    EXPECT_NEAR(r.head.position().pan, 5.0f, 1e-3f);
}

TEST(PanTiltVelocity, WatchdogRangeIsValidated)
{
    Rig r;
    EXPECT_EQ(r.head.setWatchdog(49), Result::Range);
    EXPECT_EQ(r.head.setWatchdog(5001), Result::Range);
    EXPECT_EQ(r.head.setWatchdog(50), Result::Ok);
    EXPECT_EQ(r.head.setWatchdog(5000), Result::Ok);
    EXPECT_EQ(r.head.watchdogMs(), 5000u);
}

TEST(PanTiltVelocity, WatchdogDoesNotGuardPositionMoves)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(10.0f), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    for (int i = 0; i < 20; ++i)
    {
        EXPECT_FALSE(r.head.tick(100).watchdogTripped);
    }
    EXPECT_EQ(r.head.state(), State::Moving);
}

TEST(PanTiltVelocity, ZeroVelocityIsNotWatched)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(0.0f, 0.0f), Result::Ok);
    EXPECT_FALSE(r.head.tick(2000).watchdogTripped);
}

TEST(PanTiltVelocity, MoveEndsVelocityMode)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(50.0f, 0.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    ASSERT_EQ(r.head.move(0.0f, 0.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 0.0f);
    // No longer in velocity mode: the watchdog must not fire mid-move.
    EXPECT_FALSE(r.head.tick(1000).watchdogTripped);
}

TEST(PanTiltVelocity, DisableClearsVelocityMode)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(50.0f, 50.0f), Result::Ok);
    ASSERT_EQ(r.head.disable(), Result::Ok);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_FALSE(r.head.tick(1000).watchdogTripped);
}

TEST(PanTiltVelocity, StopEndsVelocityMode)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(50.0f, 50.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(100).busOk);
    ASSERT_EQ(r.head.stop(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
}

// ── Non-finite input (IPanTilt is a plug-in boundary: any protocol can call it) ─

TEST(PanTiltRobustness, RejectsNonFiniteArguments)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    Rig         r;
    EXPECT_EQ(r.head.enableAt(nan, 0.0f), Result::Range);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.move(nan, 0.0f), Result::Range);
    EXPECT_EQ(r.head.move(0.0f, inf), Result::Range);
    EXPECT_EQ(r.head.nudge(nan, 0.0f), Result::Range);
    EXPECT_EQ(r.head.nudge(0.0f, -inf), Result::Range);
    EXPECT_EQ(r.head.setSpeed(nan), Result::Range);
    EXPECT_EQ(r.head.setSpeed(inf), Result::Range);
    EXPECT_EQ(r.head.setVelocity(nan, 0.0f), Result::Range);
    EXPECT_EQ(r.head.setVelocity(0.0f, inf), Result::Range);

    // Nothing above may have disturbed the head.
    EXPECT_FLOAT_EQ(r.head.speed(), Servo::kDefaultSpeedDegPerSec);
    EXPECT_FLOAT_EQ(r.head.target().pan, 0.0f);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_TRUE(r.head.tick(100).busOk);
    EXPECT_FLOAT_EQ(r.head.position().pan, 0.0f);
}

// ── Review findings (adversarial review 2026-10-01) ─────────────────────────

TEST(PanTiltReview, FailedDisableStillStopsMotionAndAllowsStop)  // H1
{
    Rig r;
    ASSERT_EQ(r.head.setWatchdog(0), Result::Ok);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(0.0f, 30.0f), Result::Ok);
    r.tiltCh.failNext = true;
    EXPECT_EQ(r.head.disable(), Result::Bus);
    // Tilt is still powered: the head must not claim to be disabled...
    EXPECT_NE(r.head.state(), State::Disabled);
    // ...and it must not keep moving.
    const float before = r.head.position().tilt;
    ASSERT_TRUE(r.head.tick(1000).busOk);
    EXPECT_FLOAT_EQ(r.head.position().tilt, before);
    EXPECT_EQ(r.head.stop(), Result::Ok);
    // Retrying DISABLE completes the job.
    EXPECT_EQ(r.head.disable(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Disabled);
}

TEST(PanTiltReview, PartiallyEngagedHeadRefusesNewMotionButAcceptsStop)  // H1
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    r.tiltCh.failNext = true;
    ASSERT_EQ(r.head.disable(), Result::Bus);  // pan released, tilt still on
    EXPECT_EQ(r.head.move(10.0f, 10.0f), Result::Disabled);
    EXPECT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Disabled);
    EXPECT_EQ(r.head.setVelocity(0.0f, 0.0f), Result::Ok);  // zero = stop, always allowed
    EXPECT_EQ(r.head.stop(), Result::Ok);
}

TEST(PanTiltReview, EnableAtOnPartiallyEngagedHeadDoesNotSnapEngagedAxis)  // L5
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    r.tiltCh.failNext = true;
    ASSERT_EQ(r.head.disable(), Result::Bus);  // tilt still engaged at 0
    const std::size_t tiltWrites = r.tiltCh.writes.size();
    ASSERT_EQ(r.head.enableAt(0.0f, 60.0f), Result::Ok);
    EXPECT_EQ(r.tiltCh.writes.size(), tiltWrites);  // no immediate jump to 60°
    EXPECT_FLOAT_EQ(r.head.target().tilt, 60.0f);   // slews there instead
    EXPECT_EQ(r.head.state(), State::Moving);
}

TEST(PanTiltReview, WatchdogAgeSaturatesInsteadOfWrapping)  // N4
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Ok);
    EXPECT_FALSE(r.head.tick(10).watchdogTripped);
    EXPECT_TRUE(r.head.tick(0xFFFFFFFFu - 5u).watchdogTripped);
}

TEST(PanTiltReview, LoweringTheWatchdogBelowTheCurrentAgeStillTrips)  // R3
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Ok);
    EXPECT_FALSE(r.head.tick(400).watchdogTripped);
    ASSERT_EQ(r.head.setWatchdog(50), Result::Ok);
    // 50 - 400 underflows to 2^32 - 350; this dt is just below that, and
    // 400 + dt wraps to 19 ms: a naive saturating add would not trip.
    EXPECT_TRUE(r.head.tick(0xFFFFFFFFu - 380u).watchdogTripped);
}

TEST(PanTiltReview, MotionStepIsCappedButWatchdogUsesWallTime)  // L1
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(100.0f, 0.0f), Result::Ok);
    const ::owlet::core::TickStatus st = r.head.tick(1000);  // loop starved for 1 s
    EXPECT_TRUE(st.watchdogTripped);  // 1 s of silence > 500 ms
    EXPECT_LE(r.head.position().pan, 100.0f * PanTilt::kMaxStepMs / 1000.0f + 1e-3f);
}

// ── What the head announces ─────────────────────────────────────────────────

namespace
{

using ::owlet::core::EventId;
using Ids = std::vector<EventId>;

struct AnnouncingRig
{
    FakeServoChannel                 panCh;
    FakeServoChannel                 tiltCh;
    Servo                            pan{panCh, Servo::Calibration{}};
    Servo                            tilt{tiltCh, Servo::Calibration{}};
    ::owlet::test::RecordingEventSink events;
    PanTilt                          head{pan, tilt, &events};

    AnnouncingRig() { EXPECT_EQ(head.enable(), Result::Ok); }

    // Runs the head for `ms` and returns what it announced.
    Ids run(uint32_t ms, uint32_t step = 20)
    {
        for (uint32_t t = 0; t < ms; t += step)
        {
            (void) head.tick(step);
        }
        return events.take();
    }
};

} // namespace

TEST(PanTiltEvents, AMoveIsAnnouncedOnceWhenItArrives)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.move(10.0f, -5.0f), Result::Ok);
    (void) r.head.tick(20);
    ASSERT_EQ(r.head.state(), State::Moving);
    EXPECT_TRUE(r.events.take().empty()) << "not while it is still moving";
    EXPECT_EQ(r.run(2000), Ids{EventId::Arrived});
    EXPECT_TRUE(r.run(500).empty()) << "and only once";
}

TEST(PanTiltEvents, AMoveToWhereTheHeadAlreadyIsStillArrives)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.move(0.0f, 0.0f), Result::Ok);
    EXPECT_EQ(r.run(20), Ids{EventId::Arrived}) << "a client can always wait for it";
}

TEST(PanTiltEvents, NudgeAndHomeArriveToo)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.nudge(3.0f, 0.0f), Result::Ok);
    EXPECT_EQ(r.run(1000), Ids{EventId::Arrived});
    ASSERT_EQ(r.head.home(), Result::Ok);
    EXPECT_EQ(r.run(1000), Ids{EventId::Arrived});
}

TEST(PanTiltEvents, AStoppedOrAbandonedMoveNeverArrives)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.move(40.0f, 0.0f), Result::Ok);
    (void) r.head.tick(20);
    ASSERT_EQ(r.head.stop(), Result::Ok);
    EXPECT_TRUE(r.run(500).empty());

    ASSERT_EQ(r.head.move(40.0f, 0.0f), Result::Ok);
    (void) r.head.tick(20);
    ASSERT_EQ(r.head.disable(), Result::Ok);
    EXPECT_TRUE(r.run(500).empty());

    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(-40.0f, 0.0f), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(5.0f, 0.0f), Result::Ok);
    ASSERT_EQ(r.head.setWatchdog(0), Result::Ok);
    EXPECT_TRUE(r.run(500).empty()) << "velocity mode took over";
}

TEST(PanTiltEvents, RejectedMovesAnnounceNothing)
{
    AnnouncingRig r;
    EXPECT_EQ(r.head.move(1000.0f, 0.0f), Result::Range);
    EXPECT_TRUE(r.run(100).empty());
}

TEST(PanTiltEvents, EnableAtIsAMoveOnlyWhenTheHeadIsAlreadyPowered)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.enableAt(20.0f, 0.0f), Result::Ok);  // powered: it slews there
    EXPECT_EQ(r.run(2000), Ids{EventId::Arrived});

    ASSERT_EQ(r.head.disable(), Result::Ok);
    ASSERT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Ok);   // limp: the pose is taken at once
    EXPECT_TRUE(r.run(500).empty());
}

TEST(PanTiltEvents, TheWatchdogIsAnnouncedOnce)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(r.run(480).empty());
    EXPECT_EQ(r.run(40), Ids{EventId::WatchdogExpired});
    EXPECT_TRUE(r.run(1000).empty());
}

TEST(PanTiltEvents, ABusFaultIsAnnouncedWhenItStartsAndWhenItClears)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.move(60.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(r.run(40).empty());
    r.panCh.failNext = true;
    EXPECT_EQ(r.run(20), Ids{EventId::BusFault});
    EXPECT_EQ(r.run(20), Ids{EventId::BusOk});
    EXPECT_TRUE(r.run(40).empty()) << "nothing while it stays healthy";
}

TEST(PanTiltEvents, AHeadWithNoSinkJustWorks)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(5.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(r.head.tick(1000).busOk);
    EXPECT_FLOAT_EQ(r.head.position().pan, 5.0f);
}

// ── Knowing where the head is ───────────────────────────────────────────────

namespace
{

class FakeHoming final : public ::owlet::core::IHoming
{
public:
    [[nodiscard]] bool start() override
    {
        if (running)
        {
            return false;
        }
        running = true;
        result  = Outcome::None;
        ++starts;
        return true;
    }
    void abort() override
    {
        ++aborts;
        finish(Outcome::Failed, 1, 4);
    }
    void update() override { ++updates; }
    [[nodiscard]] bool     busy() const override { return running; }
    [[nodiscard]] Outcome  outcome() const override { return result; }
    [[nodiscard]] uint8_t  failedAxis() const override { return axis; }
    [[nodiscard]] uint16_t failureCode() const override { return code; }

    void finish(Outcome o, uint8_t failedOn = 0, uint16_t why = 0)
    {
        running = false;
        result  = o;
        axis    = failedOn;
        code    = why;
    }

    bool     running{false};
    Outcome  result{Outcome::None};
    uint8_t  axis{0};
    uint16_t code{0};
    int      starts{0};
    int      aborts{0};
    int      updates{0};
};

struct TrustRig
{
    ::owlet::test::FakeAxis           pan;
    ::owlet::test::FakeAxis           tilt;
    ::owlet::test::RecordingEventSink events;
    FakeHoming                        homing;
    PanTilt                           head{pan, tilt, &events, &homing};

    TrustRig() { EXPECT_EQ(head.enable(), Result::Ok); }
};

} // namespace

TEST(PanTiltTrust, AnAxisStoppedByALimitLosesThePositionOnce)
{
    TrustRig r;
    ASSERT_EQ(r.head.move(10.0f, 0.0f), Result::Ok);
    r.tilt.isBlocked = true;
    (void) r.head.tick(2);
    (void) r.head.tick(2);
    ASSERT_EQ(r.events.events.size(), 1u) << "no arrival is claimed, and it is said once";
    EXPECT_EQ(r.events.events[0].id, EventId::PositionLost);
    EXPECT_EQ(r.events.events[0].axis, 1);
    EXPECT_FALSE(r.head.positionTrusted());
    EXPECT_EQ(r.pan.halts, 1) << "both axes stop dead";
    EXPECT_EQ(r.tilt.halts, 1);
}

TEST(PanTiltTrust, MotionIsRefusedUntilThePositionIsKnownAgain)
{
    TrustRig r;
    r.pan.isBlocked = true;
    (void) r.head.tick(2);
    EXPECT_EQ(r.head.move(1.0f, 0.0f), Result::Untrusted);
    EXPECT_EQ(r.head.nudge(1.0f, 0.0f), Result::Untrusted);
    EXPECT_EQ(r.head.home(), Result::Untrusted);
    EXPECT_EQ(r.head.setVelocity(5.0f, 0.0f), Result::Untrusted);
    EXPECT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Untrusted) << "a powered head would have to travel there";
    EXPECT_EQ(r.head.stop(), Result::Ok) << "stopping is always allowed";
    EXPECT_EQ(r.head.state(), State::Holding) << "and it keeps holding";

    // Limp, then named: the user has said where it is.
    r.pan.isBlocked = false;
    ASSERT_EQ(r.head.disable(), Result::Ok);
    ASSERT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(r.head.positionTrusted());
    EXPECT_EQ(r.head.move(1.0f, 0.0f), Result::Ok);
}

TEST(PanTiltTrust, AHeadWithoutSwitchesCannotSeek)
{
    Rig r;
    EXPECT_EQ(r.head.seekHome(), Result::Unsupported);
    EXPECT_TRUE(r.head.positionTrusted());
}

TEST(PanTiltTrust, HomingRunsFromTheTickAndIsAnnounced)
{
    TrustRig r;
    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    EXPECT_EQ(r.head.seekHome(), Result::Busy);
    EXPECT_EQ(r.head.move(1.0f, 0.0f), Result::Busy);
    EXPECT_EQ(r.head.setVelocity(1.0f, 0.0f), Result::Busy);
    EXPECT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Busy);
    r.pan.isBlocked = true;  // homing drives onto switches on purpose
    (void) r.head.tick(2);
    (void) r.head.tick(2);
    EXPECT_EQ(r.homing.updates, 2);
    EXPECT_TRUE(r.events.take().empty()) << "nothing is lost while homing is in charge";

    r.pan.isBlocked = false;
    r.homing.finish(::owlet::core::IHoming::Outcome::Done);
    (void) r.head.tick(2);
    EXPECT_EQ(r.events.take(), Ids{EventId::Homed});
    EXPECT_TRUE(r.head.positionTrusted());
    EXPECT_EQ(r.head.move(1.0f, 0.0f), Result::Ok);
}

TEST(PanTiltTrust, HomingRecoversALostPosition)
{
    TrustRig r;
    r.pan.isBlocked = true;
    (void) r.head.tick(2);
    ASSERT_FALSE(r.head.positionTrusted());
    r.pan.isBlocked = false;
    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    r.homing.finish(::owlet::core::IHoming::Outcome::Done);
    (void) r.head.tick(2);
    EXPECT_TRUE(r.head.positionTrusted());
}

TEST(PanTiltTrust, AFailedHomingLeavesThePositionUntrustedAndSaysWhy)
{
    TrustRig r;
    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    (void) r.events.take();
    r.events.events.clear();
    r.homing.finish(::owlet::core::IHoming::Outcome::Failed, 1, 2);
    (void) r.head.tick(2);
    ASSERT_EQ(r.events.events.size(), 1u);
    EXPECT_EQ(r.events.events[0].id, EventId::HomingFailed);
    EXPECT_EQ(r.events.events[0].axis, 1);
    EXPECT_EQ(r.events.events[0].arg, 2);
    EXPECT_EQ(r.head.move(1.0f, 0.0f), Result::Untrusted);
}

TEST(PanTiltTrust, StopAndDisableAbortHoming)
{
    TrustRig r;
    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    EXPECT_EQ(r.head.stop(), Result::Ok);
    EXPECT_EQ(r.homing.aborts, 1);
    EXPECT_EQ(r.events.take(), Ids{EventId::HomingFailed});
    EXPECT_FALSE(r.head.positionTrusted());

    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    EXPECT_EQ(r.head.disable(), Result::Ok);
    EXPECT_EQ(r.homing.aborts, 2);
}
