#include "FakeAxis.hpp"
#include "RecordingEventSink.hpp"
#include "ReferenceStepperHead.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/stepper/StepperAxis.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using ::owlet::StepperAxis;
using ::owlet::core::PanTilt;
using ::owlet::test::FakeAxis;
using ::owlet::test::ReferenceAxes;
using Result = ::owlet::core::PanTilt::Result;
using State = ::owlet::core::PanTilt::State;

namespace
{

constexpr uint32_t kTickMs = 2; // the Pico's control period

const float kStep = ::owlet::test::referenceStepDeg();

int64_t stepsFor(float deg)
{
    return std::lround(deg * ::owlet::test::referencePan().stepsPerDeg);
}

struct Rig : ReferenceAxes
{
    PanTilt head{pan, tilt};

    void run(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += kTickMs)
        {
            EXPECT_TRUE(head.tick(kTickMs).busOk);
        }
    }

    // Ticks until the head holds still; fails the test if it never does.
    void settle(uint32_t maxMs = 10000)
    {
        uint32_t t = 0;
        while (head.state() == State::Moving && t < maxMs)
        {
            EXPECT_TRUE(head.tick(kTickMs).busOk);
            t += kTickMs;
        }
        EXPECT_LT(t, maxMs) << "the head never came to rest";
    }
};

// For the power policy a stepper can't exercise: its driver enable never
// fails to switch, but the head must still cope with an axis whose does.
struct PowerFaultRig
{
    FakeAxis pan;
    FakeAxis tilt;
    PanTilt head{pan, tilt};
};

} // namespace

TEST(PanTilt, BootsDisabledAndLimp)
{
    Rig r;
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_FALSE(r.anyPowered());
    r.run(100);
    EXPECT_FALSE(r.anyStepped());
}

TEST(PanTilt, MotionCommandsRequireEnable)
{
    Rig r;
    EXPECT_EQ(r.head.move(10.0f, 10.0f), Result::Disabled);
    EXPECT_EQ(r.head.nudge(1.0f, 1.0f), Result::Disabled);
    EXPECT_EQ(r.head.home(), Result::Disabled);
    EXPECT_EQ(r.head.stop(), Result::Disabled);
    r.run(100);
    EXPECT_FALSE(r.anyStepped());
}

TEST(PanTilt, EnablePowersBothAxesWithoutMoving)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_TRUE(r.panEnable.on);
    EXPECT_TRUE(r.tiltEnable.on);
    r.run(100);
    EXPECT_FALSE(r.anyStepped());
}

TEST(PanTilt, EnableAtNamesThePoseWithoutMoving)
{
    Rig r;
    ASSERT_EQ(r.head.enableAt(45.0f, -45.0f), Result::Ok);
    EXPECT_NEAR(r.head.position().pan, 45.0f, kStep);
    EXPECT_NEAR(r.head.position().tilt, -45.0f, kStep);
    EXPECT_EQ(r.head.state(), State::Holding);
    r.run(100);
    EXPECT_FALSE(r.anyStepped()) << "a stepper can't snap: the pose is only named";
}

TEST(PanTilt, EnableAtOutOfRangeIsRejectedWithoutPower)
{
    Rig r;
    EXPECT_EQ(r.head.enableAt(95.0f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.enableAt(0.0f, 61.0f), Result::Range);
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_FALSE(r.anyPowered());
}

TEST(PanTilt, EnableAtWhileEnabledTravelsThere)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), Result::Ok);
    EXPECT_FALSE(r.anyStepped()) << "no jump to the new pose";
    EXPECT_EQ(r.head.state(), State::Moving);
    EXPECT_NEAR(r.head.target().pan, 60.0f, kStep);
    r.settle();
    EXPECT_EQ(r.panSteps.total, stepsFor(60.0f)) << "it went there step by step";
}

TEST(PanTilt, ReEnableAfterMidMoveDisableUsesPositionNotTarget)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    r.run(200);
    const float stoppedAt = r.head.position().pan;
    ASSERT_GT(stoppedAt, 5.0f);
    ASSERT_LT(stoppedAt, 80.0f);
    ASSERT_EQ(r.head.disable(), Result::Ok);
    EXPECT_FALSE(r.anyPowered());
    const std::size_t bursts = r.panSteps.bursts.size();
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.position().pan, stoppedAt);
    EXPECT_EQ(r.head.state(), State::Holding);
    r.run(100);
    EXPECT_EQ(r.panSteps.bursts.size(), bursts) << "the abandoned target is not resumed";
}

TEST(PanTilt, MoveRejectsOutOfLimitTargets)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.move(90.5f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.move(0.0f, -61.0f), Result::Range);
    EXPECT_EQ(r.head.move(90.0f, -60.0f), Result::Ok);
}

TEST(PanTilt, MoveRunsBothAxesIndependently)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(30.0f, -10.0f), Result::Ok);
    r.run(250);
    EXPECT_NEAR(r.head.position().tilt, -10.0f, kStep) << "the shorter move is already done";
    EXPECT_GT(r.head.position().pan, 10.0f);
    EXPECT_LT(r.head.position().pan, 30.0f);
    EXPECT_EQ(r.head.state(), State::Moving);
    r.settle();
    EXPECT_EQ(r.panSteps.total, stepsFor(30.0f));
    EXPECT_EQ(r.tiltSteps.total, stepsFor(-10.0f));
}

TEST(PanTilt, NudgeIsRelativeToTargetAndClamps)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    ASSERT_EQ(r.head.nudge(5.0f, -3.0f), Result::Ok);
    EXPECT_NEAR(r.head.target().pan, 85.0f, kStep);
    EXPECT_NEAR(r.head.target().tilt, -3.0f, kStep);
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

TEST(PanTilt, StopBrakesToRestShortOfTheTarget)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(50.0f, 50.0f), Result::Ok);
    r.run(150); // up to speed, nowhere near 50°
    const float at = r.head.position().pan;
    ASSERT_EQ(r.head.stop(), Result::Ok);
    // A stepper stalls if its rate jumps, so it rests a braking distance on.
    EXPECT_GT(r.head.target().pan, at);
    EXPECT_LT(r.head.target().pan, 50.0f);
    const float restsAt = r.head.target().pan;
    r.settle();
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_FLOAT_EQ(r.head.position().pan, restsAt);
}

TEST(PanTilt, SpeedRangeComesFromTheAxes)
{
    Rig r;
    EXPECT_EQ(r.head.setSpeed(0.5f), Result::Range);
    EXPECT_EQ(r.head.setSpeed(241.0f), Result::Range);
    EXPECT_EQ(r.head.setSpeed(240.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.speed(), 240.0f);
    EXPECT_FLOAT_EQ(r.pan.speed(), 240.0f);
    EXPECT_FLOAT_EQ(r.tilt.speed(), 240.0f);
}

TEST(PanTilt, EnableFailureLeavesBothAxesLimp)
{
    PowerFaultRig r;
    r.tilt.failEngage = true;
    EXPECT_EQ(r.head.enable(), Result::Bus);
    EXPECT_EQ(r.head.state(), State::Disabled);
    EXPECT_FALSE(r.pan.on);
}

TEST(PanTilt, ReportsLimitsFromTheAxes)
{
    ::owlet::test::FakeStepGenerator a;
    ::owlet::test::FakeStepGenerator b;
    ::owlet::test::FakeDigitalOutput aOn;
    ::owlet::test::FakeDigitalOutput bOn;
    StepperAxis::Config panCfg = ::owlet::test::referencePan();
    panCfg.minDeg = -60.0f;
    panCfg.maxDeg = 70.0f;
    StepperAxis::Config tiltCfg = ::owlet::test::referenceTilt();
    tiltCfg.minDeg = -30.0f;
    StepperAxis pan{a, aOn, panCfg};
    StepperAxis tilt{b, bOn, tiltCfg};
    PanTilt head{pan, tilt};
    const PanTilt::Limits lim = head.limits();
    EXPECT_FLOAT_EQ(lim.panMin, -60.0f);
    EXPECT_FLOAT_EQ(lim.panMax, 70.0f);
    EXPECT_FLOAT_EQ(lim.tiltMin, -30.0f);
    EXPECT_FLOAT_EQ(lim.tiltMax, 60.0f);
}

TEST(PanTilt, TickReportsAStalledStepGenerator)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(10.0f, 0.0f), Result::Ok);
    r.run(20);
    r.panSteps.busy = true;
    // A refused burst is retried; only a generator that stays refusing is a fault.
    uint32_t refusedMs = 0;
    while (r.head.tick(kTickMs).busOk && refusedMs < 1000)
    {
        refusedMs += kTickMs;
    }
    refusedMs += kTickMs;
    EXPECT_EQ(refusedMs, StepperAxis::kStallReportMs);
    r.panSteps.busy = false;
    EXPECT_TRUE(r.head.tick(kTickMs).busOk);
    r.settle();
    EXPECT_EQ(r.panSteps.total, stepsFor(10.0f)) << "no step was lost while it was refused";
}

TEST(PanTilt, UsableThroughInterfaces)
{
    Rig r;
    ::owlet::core::IPanTilt& control = r.head;
    ::owlet::core::ITickable& motion = r.head;
    ASSERT_EQ(control.enable(), Result::Ok);
    ASSERT_EQ(control.move(5.0f, 0.0f), Result::Ok);
    for (int i = 0; i < 100; ++i)
    {
        EXPECT_TRUE(motion.tick(10).busOk);
    }
    EXPECT_NEAR(control.position().pan, 5.0f, kStep);
}

// ── Velocity mode + watchdog ────────────────────────────────────────────────

TEST(PanTiltVelocity, RequiresEnable)
{
    Rig r;
    EXPECT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Disabled);
}

TEST(PanTiltVelocity, RangeComesFromTheAxes)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_EQ(r.head.setVelocity(240.5f, 0.0f), Result::Range);
    EXPECT_EQ(r.head.setVelocity(0.0f, -241.0f), Result::Range);
    EXPECT_EQ(r.head.setVelocity(240.0f, -240.0f), Result::Ok);
}

TEST(PanTiltVelocity, MovesBothAxes)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(20.0f, -10.0f), Result::Ok);
    r.run(100);
    // 0.1 s at the rate, less what the ramp up to it costs (v² / 2a).
    const float accel = ::owlet::test::referencePan().accelDegPerSec2;
    EXPECT_NEAR(r.head.position().pan, 2.0f - 20.0f * 20.0f / (2.0f * accel), 2.0f * kStep);
    EXPECT_NEAR(r.head.position().tilt, -1.0f + 10.0f * 10.0f / (2.0f * accel), 2.0f * kStep);
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
    EXPECT_TRUE(r.head.tick(100).watchdogTripped); // 500 ms without a refresh
    bool trippedAgain = false;
    for (int i = 0; i < 100; ++i)
    {
        trippedAgain = r.head.tick(kTickMs).watchdogTripped || trippedAgain;
    }
    EXPECT_FALSE(trippedAgain) << "reported once";
    EXPECT_EQ(r.head.state(), State::Holding); // braked to rest, still enabled
    const float here = r.head.position().pan;
    EXPECT_FALSE(r.head.tick(1000).watchdogTripped);
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
    EXPECT_NEAR(r.head.position().pan, 5.0f, kStep);
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
    r.settle();
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_FALSE(r.head.tick(1000).watchdogTripped) << "nothing left for the watchdog to stop";
}

// ── Non-finite input (IPanTilt is a plug-in boundary: any protocol can call it) ─

TEST(PanTiltRobustness, RejectsNonFiniteArguments)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    Rig r;
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
    EXPECT_FLOAT_EQ(r.head.speed(), StepperAxis::kDefaultSpeedDegPerSec);
    EXPECT_FLOAT_EQ(r.head.target().pan, 0.0f);
    EXPECT_EQ(r.head.state(), State::Holding);
    r.run(100);
    EXPECT_FALSE(r.anyStepped());
}

// ── Review findings (adversarial review 2026-10-01) ─────────────────────────

TEST(PanTiltReview, FailedDisableIsNotReportedAsDisabledAndStopStillWorks) // H1
{
    PowerFaultRig r;
    ASSERT_EQ(r.head.setWatchdog(0), Result::Ok);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(0.0f, 30.0f), Result::Ok);
    r.tilt.failRelease = true;
    EXPECT_EQ(r.head.disable(), Result::Bus);
    // Tilt is still powered: the head must not claim to be disabled...
    EXPECT_NE(r.head.state(), State::Disabled);
    // ...and STOP must still reach it.
    EXPECT_EQ(r.head.stop(), Result::Ok);
    EXPECT_FALSE(r.tilt.inVelocityMode());
    // Retrying DISABLE completes the job.
    r.tilt.failRelease = false;
    EXPECT_EQ(r.head.disable(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Disabled);
}

TEST(PanTiltReview, PartiallyEngagedHeadRefusesNewMotionButAcceptsStop) // H1
{
    PowerFaultRig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    r.tilt.failRelease = true;
    ASSERT_EQ(r.head.disable(), Result::Bus); // pan released, tilt still on
    EXPECT_EQ(r.head.move(10.0f, 10.0f), Result::Disabled);
    EXPECT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Disabled);
    EXPECT_EQ(r.head.setVelocity(0.0f, 0.0f), Result::Ok); // zero = stop, always allowed
    EXPECT_EQ(r.head.stop(), Result::Ok);
}

TEST(PanTiltReview, EnableAtOnPartiallyEngagedHeadDoesNotSnapEngagedAxis) // L5
{
    PowerFaultRig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    r.tilt.failRelease = true;
    ASSERT_EQ(r.head.disable(), Result::Bus); // tilt still engaged at 0
    ASSERT_EQ(r.head.enableAt(0.0f, 60.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.tilt.position, 0.0f);       // not renamed or jumped to 60°
    EXPECT_FLOAT_EQ(r.head.target().tilt, 60.0f); // it travels there instead
    EXPECT_EQ(r.head.state(), State::Moving);
}

TEST(PanTiltReview, WatchdogAgeSaturatesInsteadOfWrapping) // N4
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), Result::Ok);
    EXPECT_FALSE(r.head.tick(10).watchdogTripped);
    EXPECT_TRUE(r.head.tick(0xFFFFFFFFu - 5u).watchdogTripped);
}

TEST(PanTiltReview, LoweringTheWatchdogBelowTheCurrentAgeStillTrips) // R3
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

TEST(PanTiltReview, MotionStepIsCappedButWatchdogUsesWallTime) // L1
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setVelocity(100.0f, 0.0f), Result::Ok);
    const ::owlet::core::TickStatus st = r.head.tick(1000); // loop starved for 1 s
    EXPECT_TRUE(st.watchdogTripped);                        // 1 s of silence > 500 ms
    EXPECT_LE(r.head.position().pan, 100.0f * PanTilt::kMaxStepMs / 1000.0f + 1e-3f);
}

// ── What the head announces ─────────────────────────────────────────────────

namespace
{

using ::owlet::core::EventId;
using Ids = std::vector<EventId>;

struct AnnouncingRig : ReferenceAxes
{
    ::owlet::test::RecordingEventSink events;
    PanTilt head{pan, tilt, &events};

    AnnouncingRig()
    {
        EXPECT_EQ(head.enable(), Result::Ok);
    }

    // Runs the head for `ms` and returns what it announced.
    Ids run(uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += kTickMs)
        {
            (void) head.tick(kTickMs);
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
    EXPECT_TRUE(r.run(500).empty()) << "braking to rest is not arriving";

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
    ASSERT_EQ(r.head.enableAt(20.0f, 0.0f), Result::Ok); // powered: it travels there
    EXPECT_EQ(r.run(2000), Ids{EventId::Arrived});

    ASSERT_EQ(r.head.disable(), Result::Ok);
    ASSERT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Ok); // limp: the pose is taken at once
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

TEST(PanTiltEvents, AStalledStepGeneratorIsAnnouncedWhenItStartsAndWhenItClears)
{
    AnnouncingRig r;
    ASSERT_EQ(r.head.move(60.0f, 0.0f), Result::Ok);
    EXPECT_TRUE(r.run(40).empty());
    r.panSteps.busy = true;
    EXPECT_EQ(r.run(StepperAxis::kStallReportMs + 20), Ids{EventId::BusFault});
    r.panSteps.busy = false;
    EXPECT_EQ(r.run(20), Ids{EventId::BusOk});
    EXPECT_TRUE(r.run(40).empty()) << "nothing while it stays healthy";
}

TEST(PanTiltEvents, AHeadWithNoSinkJustWorks)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(5.0f, 0.0f), Result::Ok);
    r.settle();
    EXPECT_NEAR(r.head.position().pan, 5.0f, kStep);
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
        result = Outcome::None;
        ++starts;
        return true;
    }
    void abort() override
    {
        ++aborts;
        finish(Outcome::Failed, 1, 4);
    }
    void update() override
    {
        ++updates;
    }
    [[nodiscard]] bool busy() const override
    {
        return running;
    }
    [[nodiscard]] Outcome outcome() const override
    {
        return result;
    }
    [[nodiscard]] uint8_t failedAxis() const override
    {
        return axis;
    }
    [[nodiscard]] uint16_t failureCode() const override
    {
        return code;
    }

    void finish(Outcome o, uint8_t failedOn = 0, uint16_t why = 0)
    {
        running = false;
        result = o;
        axis = failedOn;
        code = why;
    }

    bool running{false};
    Outcome result{Outcome::None};
    uint8_t axis{0};
    uint16_t code{0};
    int starts{0};
    int aborts{0};
    int updates{0};
};

struct TrustRig
{
    FakeAxis pan;
    FakeAxis tilt;
    ::owlet::test::RecordingEventSink events;
    FakeHoming homing;
    PanTilt head{pan, tilt, &events, &homing};

    TrustRig()
    {
        EXPECT_EQ(head.enable(), Result::Ok);
    }
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
    EXPECT_EQ(r.head.enableAt(0.0f, 0.0f), Result::Untrusted)
        << "a powered head would have to travel there";
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
    r.pan.isBlocked = true; // homing drives onto switches on purpose
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
