#include "FakeAxis.hpp"
#include "FakeCalibrationStore.hpp"
#include "FakeDigitalOutput.hpp"
#include "FakeStepGenerator.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/stepper/SharedEnable.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>

using ::owlet::SharedEnable;
using ::owlet::StepperAxis;
using ::owlet::hal::AxisCalibration;
using ::owlet::test::FakeAxis;
using ::owlet::test::FakeDigitalOutput;
using ::owlet::test::FakeStepGenerator;

namespace
{

constexpr uint32_t kTickMs = 2;

// The belt head: 200 full steps x 16 microsteps x 3:1 = 9600 per revolution.
StepperAxis::Config beltConfig()
{
    StepperAxis::Config c;
    c.stepsPerDeg = 9600.0f / 360.0f;
    c.maxSpeedDegPerSec = 180.0f;
    c.accelDegPerSec2 = 720.0f;
    c.minDeg = -90.0f;
    c.maxDeg = 90.0f;
    c.travelMinDeg = -170.0f;
    c.travelMaxDeg = 170.0f;
    return c;
}

struct Rig
{
    explicit Rig(const StepperAxis::Config& cfg = beltConfig()) : axis(gen, enable, cfg) {}

    // Tick until the axis rests; fails the test if it never does.
    int settle(int maxTicks = 20000)
    {
        int n = 0;
        while (axis.moving() && n < maxTicks)
        {
            EXPECT_TRUE(axis.tick(kTickMs));
            ++n;
        }
        EXPECT_LT(n, maxTicks) << "axis never came to rest";
        return n;
    }

    FakeStepGenerator gen;
    FakeDigitalOutput enable;
    StepperAxis axis;
};

int32_t stepsFor(float deg)
{
    return static_cast<int32_t>(std::lround(deg * beltConfig().stepsPerDeg));
}

} // namespace

// ── Power ───────────────────────────────────────────────────────────────────

TEST(StepperAxis, BootsReleasedAndSilent)
{
    Rig r;
    EXPECT_FALSE(r.axis.engaged());
    EXPECT_FALSE(r.axis.moving());
    EXPECT_FALSE(r.enable.on);
    r.axis.setTarget(30.0f);
    EXPECT_TRUE(r.axis.tick(kTickMs));
    EXPECT_TRUE(r.gen.bursts.empty()) << "a released axis must never step";
}

TEST(StepperAxis, EngageAdoptsThePositionWithoutStepping)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(20.0f));
    EXPECT_TRUE(r.axis.engaged());
    EXPECT_TRUE(r.enable.on);
    EXPECT_NEAR(r.axis.positionDeg(), 20.0f, 0.02f);
    EXPECT_NEAR(r.axis.targetDeg(), 20.0f, 0.02f);
    EXPECT_FALSE(r.axis.moving());
    EXPECT_TRUE(r.gen.bursts.empty()) << "a stepper cannot snap: engaging only names where it is";
}

TEST(StepperAxis, ReleaseCutsTheDriverAndStopsStepping)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(60.0f);
    for (int i = 0; i < 50; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    const int64_t before = r.gen.total;
    ASSERT_GT(before, 0);
    ASSERT_TRUE(r.axis.release());
    EXPECT_FALSE(r.enable.on);
    EXPECT_FALSE(r.axis.engaged());
    EXPECT_FALSE(r.axis.moving());
    for (int i = 0; i < 50; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    EXPECT_EQ(r.gen.total, before);
}

// ── Position moves ──────────────────────────────────────────────────────────

TEST(StepperAxis, MoveLandsExactlyOnTheTarget)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(45.0f);
    EXPECT_TRUE(r.axis.moving());
    r.settle();
    EXPECT_EQ(r.gen.total, stepsFor(45.0f));
    EXPECT_NEAR(r.axis.positionDeg(), 45.0f, 0.02f);
    r.axis.setTarget(-30.0f);
    r.settle();
    EXPECT_EQ(r.gen.total, stepsFor(-30.0f));
}

TEST(StepperAxis, NeverExceedsTheCommandedSpeed)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(-80.0f));
    r.axis.setSpeed(60.0f);
    r.axis.setTarget(80.0f);
    r.settle();
    const float maxPerTick = 60.0f * beltConfig().stepsPerDeg * (kTickMs / 1000.0f);
    for (const FakeStepGenerator::Burst& b : r.gen.bursts)
    {
        EXPECT_LE(std::abs(b.steps), static_cast<int32_t>(std::ceil(maxPerTick)) + 1);
        EXPECT_EQ(b.durationUs, kTickMs * 1000u);
    }
}

TEST(StepperAxis, RampsUpAndDownInsteadOfJumping)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(-80.0f));
    r.axis.setSpeed(180.0f);
    r.axis.setTarget(80.0f);
    // A stepper stalls on a rate jump, so the per-tick step count may only
    // change by what the acceleration allows (plus rounding to whole steps).
    const float dt = kTickMs / 1000.0f;
    const float maxDelta = 720.0f * beltConfig().stepsPerDeg * dt * dt;
    int32_t previous = 0;
    int32_t peak = 0;
    int guard = 0;
    while (r.axis.moving() && guard++ < 20000)
    {
        const std::size_t before = r.gen.bursts.size();
        ASSERT_TRUE(r.axis.tick(kTickMs));
        const int32_t now = (r.gen.bursts.size() > before) ? r.gen.bursts.back().steps : 0;
        EXPECT_LE(std::abs(now - previous), static_cast<int32_t>(std::ceil(maxDelta)) + 1);
        peak = std::max(peak, now);
        previous = now;
    }
    EXPECT_LE(previous, 2) << "must arrive slowly, not at full speed";
    EXPECT_GT(peak, 5) << "must actually get up to speed";
    EXPECT_EQ(r.gen.total, stepsFor(80.0f) - stepsFor(-80.0f));
}

TEST(StepperAxis, ReversingMidMoveDeceleratesThroughZero)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(80.0f);
    for (int i = 0; i < 200; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    ASSERT_GT(r.gen.bursts.back().steps, 3);
    r.axis.setTarget(-10.0f);
    ASSERT_TRUE(r.axis.tick(kTickMs));
    EXPECT_GT(r.gen.bursts.back().steps, 0) << "still coasting forward right after the reversal";
    r.settle();
    EXPECT_EQ(r.gen.total, stepsFor(-10.0f));
}

TEST(StepperAxis, ReversedAxisStepsTheOtherWay)
{
    StepperAxis::Config cfg = beltConfig();
    cfg.reversed = true;
    Rig r(cfg);
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(10.0f);
    r.settle();
    EXPECT_EQ(r.gen.total, -stepsFor(10.0f));
    EXPECT_NEAR(r.axis.positionDeg(), 10.0f, 0.02f);
}

TEST(StepperAxis, TargetsAreClampedAndNanGoesToCentre)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(500.0f);
    EXPECT_NEAR(r.axis.targetDeg(), 90.0f, 0.02f);
    r.axis.setTarget(std::numeric_limits<float>::quiet_NaN());
    EXPECT_NEAR(r.axis.targetDeg(), 0.0f, 0.02f);
}

TEST(StepperAxis, ABusyGeneratorLosesNoSteps)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(20.0f);
    for (int i = 0; i < 4000 && r.axis.moving(); ++i)
    {
        r.gen.busy = (i % 3 == 1); // refuse every third burst
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    EXPECT_FALSE(r.axis.moving());
    EXPECT_GT(r.gen.refused, 0);
    EXPECT_EQ(r.gen.total, stepsFor(20.0f)) << "what was emitted must equal what was counted";
}

TEST(StepperAxis, AGeneratorThatNeverAcceptsIsReportedAsAFault)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(20.0f);
    r.gen.busy = true; // e.g. the pulse hardware failed to start
    bool reported = false;
    for (int i = 0; i < 500 && !reported; ++i) // one second
    {
        reported = !r.axis.tick(kTickMs);
    }
    EXPECT_TRUE(reported) << "a head that silently never moves is the worst failure";
    EXPECT_EQ(r.gen.total, 0);
    EXPECT_NEAR(r.axis.positionDeg(), 0.0f, 0.02f) << "nothing went out, so nothing is counted";

    r.gen.busy = false; // recovers by itself once bursts are accepted again
    EXPECT_TRUE(r.axis.tick(kTickMs));
    r.settle();
    EXPECT_EQ(r.gen.total, stepsFor(20.0f));
}

TEST(StepperAxis, AnAbsurdTickLengthCannotOverflowTheStepCount)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setVelocity(180.0f);
    for (int i = 0; i < 200; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    const std::size_t before = r.gen.bursts.size();
    ASSERT_TRUE(r.axis.tick(0xFFFFFFFFu)); // a wrapped or corrupted dt
    ASSERT_EQ(r.gen.bursts.size(), before + 1);
    // Never more than StepperAxis::kMaxTickMs worth of motion in one burst.
    const float limit =
        180.0f * beltConfig().stepsPerDeg * (static_cast<float>(StepperAxis::kMaxTickMs) / 1000.0f);
    EXPECT_LE(r.gen.bursts.back().steps, static_cast<int32_t>(limit) + 1);
    EXPECT_EQ(r.gen.bursts.back().durationUs, StepperAxis::kMaxTickMs * 1000u);
}

// ── Velocity mode ───────────────────────────────────────────────────────────

TEST(StepperAxis, VelocityModeRunsAtTheRateAndStopsOnTheSoftLimit)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setVelocity(90.0f);
    EXPECT_TRUE(r.axis.inVelocityMode());
    for (int i = 0; i < 250; ++i) // 0.5 s: well past the ramp, short of the limit
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    const float cruise = 90.0f * beltConfig().stepsPerDeg * (kTickMs / 1000.0f);
    EXPECT_NEAR(static_cast<float>(r.gen.bursts.back().steps), cruise, 1.01f);
    EXPECT_NEAR(r.axis.targetDeg(), r.axis.positionDeg(), 1e-3f) << "the target follows the head";
    r.settle();
    EXPECT_EQ(r.gen.total, stepsFor(90.0f)) << "exactly on the limit, never past it";
    EXPECT_TRUE(r.axis.inVelocityMode());
    EXPECT_FALSE(r.axis.moving()) << "pinned at the limit";
}

TEST(StepperAxis, VelocityIsClampedAndNanStops)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setVelocity(-100000.0f);
    for (int i = 0; i < 400; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    const float maxPerTick = 180.0f * beltConfig().stepsPerDeg * (kTickMs / 1000.0f);
    EXPECT_GE(r.gen.bursts.back().steps, -static_cast<int32_t>(std::ceil(maxPerTick)) - 1);
    r.axis.setVelocity(std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(r.axis.inVelocityMode());
}

TEST(StepperAxis, StopDeceleratesToRestInsteadOfHalting)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setVelocity(120.0f);
    for (int i = 0; i < 200; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    const int64_t atStop = r.gen.total;
    r.axis.stop();
    EXPECT_FALSE(r.axis.inVelocityMode());
    EXPECT_TRUE(r.axis.moving()) << "a running stepper needs room to stop";
    const int ticks = r.settle();
    EXPECT_GT(ticks, 10);
    // Stopping distance is v²/2a = 120²/1440 = 10°.
    EXPECT_NEAR(static_cast<float>(r.gen.total - atStop) / beltConfig().stepsPerDeg, 10.0f, 1.0f);
    EXPECT_NEAR(r.axis.targetDeg(), r.axis.positionDeg(), 1e-3f);
}

// ── Capabilities and calibration ────────────────────────────────────────────

TEST(StepperAxis, CapabilitiesComeFromTheConfig)
{
    Rig r;
    const auto caps = r.axis.capabilities();
    EXPECT_FLOAT_EQ(caps.speedMin, StepperAxis::kMinSpeedDegPerSec);
    EXPECT_FLOAT_EQ(caps.speedMax, 180.0f);
    EXPECT_FLOAT_EQ(caps.velocityMax, 180.0f);
    r.axis.setSpeed(9999.0f);
    EXPECT_FLOAT_EQ(r.axis.speed(), 180.0f);
    r.axis.setSpeed(std::numeric_limits<float>::quiet_NaN());
    EXPECT_FLOAT_EQ(r.axis.speed(), 180.0f);
}

TEST(StepperAxis, CalibrationIsLimitsOnly)
{
    Rig r;
    const AxisCalibration now = r.axis.calibration();
    EXPECT_EQ(now.driveMin, 0);
    EXPECT_EQ(now.driveCenter, 0);
    EXPECT_EQ(now.driveMax, 0);
    EXPECT_FLOAT_EQ(now.minDeg, -90.0f);
    EXPECT_FLOAT_EQ(now.maxDeg, 90.0f);

    EXPECT_TRUE(r.axis.acceptsCalibration(AxisCalibration{0, 0, 0, -150.0f, 120.0f}));
    EXPECT_FALSE(r.axis.acceptsCalibration(AxisCalibration{500, 1500, 2500, -90.0f, 90.0f}))
        << "drive numbers mean nothing to a stepper";
    EXPECT_FALSE(r.axis.acceptsCalibration(AxisCalibration{0, 0, 0, -171.0f, 90.0f}));
    EXPECT_FALSE(r.axis.acceptsCalibration(AxisCalibration{0, 0, 0, 10.0f, 90.0f}));
    EXPECT_FALSE(r.axis.acceptsCalibration(
        AxisCalibration{0, 0, 0, std::numeric_limits<float>::quiet_NaN(), 90.0f}));
    EXPECT_FALSE(r.axis.calibrationNeedsRelease(AxisCalibration{0, 0, 0, -45.0f, 45.0f}));
    EXPECT_TRUE(r.axis.acceptsCalibration(r.axis.defaultCalibration()));
}

TEST(StepperAxis, NarrowedLimitsBringThePoweredAxisBackInsideWithoutAJump)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(80.0f);
    r.settle();
    const std::size_t burstsBefore = r.gen.bursts.size();
    r.axis.applyCalibration(AxisCalibration{0, 0, 0, -45.0f, 45.0f});
    EXPECT_FALSE(r.axis.positionInLimits());
    EXPECT_EQ(r.gen.bursts.size(), burstsBefore) << "applying a calibration never steps by itself";
    EXPECT_TRUE(r.axis.moving());
    r.settle();
    EXPECT_NEAR(r.axis.positionDeg(), 45.0f, 0.02f);
    EXPECT_TRUE(r.axis.positionInLimits());
}

// ── Under the unchanged core ────────────────────────────────────────────────

TEST(StepperHead, TheCoreDrivesSteppersWithNoChanges)
{
    using ::owlet::core::IPanTilt;
    FakeStepGenerator panGen;
    FakeStepGenerator tiltGen;
    FakeDigitalOutput line;
    SharedEnable enable(line);
    SharedEnable::User panEn(enable);
    SharedEnable::User tiltEn(enable);
    StepperAxis pan(panGen, panEn, beltConfig());
    StepperAxis tilt(tiltGen, tiltEn, beltConfig());
    ::owlet::core::PanTilt head(pan, tilt);

    EXPECT_EQ(head.state(), IPanTilt::State::Disabled);
    EXPECT_EQ(head.move(10.0f, 5.0f), IPanTilt::Result::Disabled);
    EXPECT_FALSE(line.on) << "boot limp: drivers stay off until ENABLE";

    ASSERT_EQ(head.enable(), IPanTilt::Result::Ok);
    EXPECT_TRUE(line.on);
    EXPECT_EQ(head.state(), IPanTilt::State::Holding);
    ASSERT_EQ(head.move(30.0f, -15.0f), IPanTilt::Result::Ok);
    EXPECT_EQ(head.state(), IPanTilt::State::Moving);
    for (int i = 0; i < 20000 && head.state() == IPanTilt::State::Moving; ++i)
    {
        EXPECT_TRUE(head.tick(kTickMs).busOk);
    }
    EXPECT_EQ(head.state(), IPanTilt::State::Holding);
    EXPECT_EQ(panGen.total, stepsFor(30.0f));
    EXPECT_EQ(tiltGen.total, stepsFor(-15.0f));
    EXPECT_FLOAT_EQ(head.capabilities().speedMax, 180.0f);
    EXPECT_EQ(head.setSpeed(181.0f), IPanTilt::Result::Range);

    ASSERT_EQ(head.disable(), IPanTilt::Result::Ok);
    EXPECT_FALSE(line.on);
}

TEST(StepperHead, WatchdogStopsARunawayVelocityCommand)
{
    using ::owlet::core::IPanTilt;
    FakeStepGenerator panGen;
    FakeStepGenerator tiltGen;
    FakeDigitalOutput en;
    StepperAxis::Config wide = beltConfig();
    wide.minDeg = -170.0f;
    wide.maxDeg = 170.0f;
    StepperAxis pan(panGen, en, wide);
    StepperAxis tilt(tiltGen, en, wide);
    ::owlet::core::PanTilt head(pan, tilt);
    ASSERT_EQ(head.enable(), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setWatchdog(100), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setVelocity(60.0f, 0.0f), IPanTilt::Result::Ok);
    bool tripped = false;
    for (int i = 0; i < 2000; ++i)
    {
        tripped = head.tick(kTickMs).watchdogTripped || tripped;
    }
    EXPECT_TRUE(tripped);
    EXPECT_EQ(head.state(), IPanTilt::State::Holding);
    EXPECT_LT(head.position().pan, 20.0f) << "0.1 s at 60°/s plus the ramp down";
}

TEST(StepperHead, CalibratorPersistsStepperLimits)
{
    using ::owlet::core::Axis;
    using ::owlet::core::ICalibration;
    FakeStepGenerator gen;
    FakeDigitalOutput en;
    StepperAxis pan(gen, en, beltConfig());
    StepperAxis tilt(gen, en, beltConfig());
    ::owlet::core::PanTilt head(pan, tilt);
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator calib(
        pan, tilt, head, store, {pan.defaultCalibration(), tilt.defaultCalibration()});
    calib.begin();
    EXPECT_TRUE(calib.factoryWasValid());
    EXPECT_EQ(calib.set(Axis::Pan, AxisCalibration{0, 0, 0, -120.0f, 120.0f}),
              ICalibration::Result::Ok);
    EXPECT_EQ(calib.set(Axis::Tilt, AxisCalibration{500, 1500, 2500, -45.0f, 45.0f}),
              ICalibration::Result::Range);
    EXPECT_EQ(calib.save(), ICalibration::Result::Ok);
    EXPECT_FLOAT_EQ(store.record.pan.maxDeg, 120.0f);
    EXPECT_FLOAT_EQ(head.limits().panMax, 120.0f);
}

TEST(PanTiltAnyMotor, CapabilitiesAreWhatBothAxesCanDo)
{
    FakeAxis fast;
    FakeAxis slow;
    fast.caps = {1.0f, 400.0f, 400.0f};
    slow.caps = {5.0f, 90.0f, 60.0f};
    ::owlet::core::PanTilt head(fast, slow);
    const auto caps = head.capabilities();
    EXPECT_FLOAT_EQ(caps.speedMin, 5.0f);
    EXPECT_FLOAT_EQ(caps.speedMax, 90.0f);
    EXPECT_FLOAT_EQ(caps.velocityMax, 60.0f);
    ASSERT_EQ(head.enable(), ::owlet::core::IPanTilt::Result::Ok);
    EXPECT_EQ(head.setSpeed(91.0f), ::owlet::core::IPanTilt::Result::Range);
    EXPECT_EQ(head.setVelocity(61.0f, 0.0f), ::owlet::core::IPanTilt::Result::Range);
    EXPECT_EQ(head.setVelocity(60.0f, -60.0f), ::owlet::core::IPanTilt::Result::Ok);
}

// ── Shared enable line and microstep select ─────────────────────────────────

TEST(SharedEnable, LineIsOnWhileAnyUserWantsIt)
{
    FakeDigitalOutput line;
    SharedEnable shared(line);
    SharedEnable::User a(shared);
    SharedEnable::User b(shared);
    EXPECT_FALSE(line.on);
    a.set(true);
    EXPECT_TRUE(line.on);
    b.set(true);
    a.set(true); // repeated requests don't count twice
    a.set(false);
    EXPECT_TRUE(line.on) << "b still needs it";
    b.set(false);
    EXPECT_FALSE(line.on);
    b.set(false);
    a.set(true);
    EXPECT_TRUE(line.on);
}

TEST(Tmc2209Standalone, MicrostepPinsFollowTheDatasheetTable)
{
    using ::owlet::tmc2209::Microsteps;
    struct Row
    {
        Microsteps mode;
        bool ms1;
        bool ms2;
        uint32_t count;
    };
    for (const Row& row : {Row{Microsteps::k8, false, false, 8},
                           Row{Microsteps::k32, true, false, 32},
                           Row{Microsteps::k64, false, true, 64},
                           Row{Microsteps::k16, true, true, 16}})
    {
        FakeDigitalOutput ms1;
        FakeDigitalOutput ms2;
        ::owlet::tmc2209::selectMicrosteps(ms1, ms2, row.mode);
        EXPECT_EQ(ms1.on, row.ms1);
        EXPECT_EQ(ms2.on, row.ms2);
        EXPECT_EQ(::owlet::tmc2209::microstepCount(row.mode), row.count);
    }
    // 200 full steps, 1/16, 3:1 belt: 9600 per turn of the output.
    EXPECT_FLOAT_EQ(::owlet::tmc2209::stepsPerDegree(200, Microsteps::k16, 3.0f), 9600.0f / 360.0f);
}

// ── Limit switches and halting ──────────────────────────────────────────────

namespace
{

class FakeSwitch final : public ::owlet::hal::IDigitalInput
{
public:
    [[nodiscard]] bool active() const override
    {
        return pressed;
    }
    bool pressed{false};
};

struct SwitchRig
{
    SwitchRig() : axis(gen, enable, beltConfig(), StepperAxis::LimitSwitches{&minSwitch, &maxSwitch})
    {
        EXPECT_TRUE(axis.engageAt(0.0f));
    }

    void run(int ticks)
    {
        for (int i = 0; i < ticks; ++i)
        {
            (void)axis.tick(kTickMs);
        }
    }

    FakeStepGenerator gen;
    FakeDigitalOutput enable;
    FakeSwitch minSwitch;
    FakeSwitch maxSwitch;
    StepperAxis axis;
};

} // namespace

TEST(StepperLimits, APressedSwitchStopsStepsTowardItAtOnce)
{
    SwitchRig r;
    r.axis.setTarget(60.0f);
    r.run(100);
    ASSERT_TRUE(r.axis.moving());
    const int64_t before = r.gen.total;
    ASSERT_GT(before, 0);

    r.maxSwitch.pressed = true;
    r.run(50);
    EXPECT_EQ(r.gen.total, before) << "not one more step, and no braking ramp";
    EXPECT_TRUE(r.axis.blocked());
    EXPECT_FALSE(r.axis.moving()) << "the move is abandoned, not paused";
    EXPECT_FLOAT_EQ(r.axis.targetDeg(), r.axis.positionDeg());

    r.maxSwitch.pressed = false;
    r.run(50);
    EXPECT_EQ(r.gen.total, before) << "releasing the switch does not restart the move";
}

TEST(StepperLimits, StepsAwayFromAPressedSwitchAreAllowed)
{
    SwitchRig r;
    r.maxSwitch.pressed = true;
    r.axis.setTarget(-10.0f);
    r.run(2000);
    EXPECT_FALSE(r.axis.blocked());
    EXPECT_NEAR(r.axis.positionDeg(), -10.0f, 0.05f);
}

TEST(StepperLimits, EachSwitchGuardsItsOwnDirection)
{
    SwitchRig r;
    r.minSwitch.pressed = true;
    r.axis.setTarget(-10.0f);
    r.run(100);
    EXPECT_EQ(r.gen.total, 0);
    EXPECT_TRUE(r.axis.blocked());

    r.axis.setTarget(10.0f);
    r.run(2000);
    EXPECT_NEAR(r.axis.positionDeg(), 10.0f, 0.05f);
}

TEST(StepperLimits, VelocityModeIsStoppedTheSameWay)
{
    SwitchRig r;
    r.axis.setVelocity(30.0f);
    r.run(100);
    r.maxSwitch.pressed = true;
    const int64_t before = r.gen.total;
    r.run(100);
    EXPECT_EQ(r.gen.total, before);
    EXPECT_FALSE(r.axis.inVelocityMode());
}

TEST(StepperLimits, AReversedAxisStillGuardsTheLogicalDirection)
{
    StepperAxis::Config cfg = beltConfig();
    cfg.reversed = true;
    FakeStepGenerator gen;
    FakeDigitalOutput enable;
    FakeSwitch minSwitch;
    FakeSwitch maxSwitch;
    StepperAxis axis(gen, enable, cfg, StepperAxis::LimitSwitches{&minSwitch, &maxSwitch});
    ASSERT_TRUE(axis.engageAt(0.0f));
    maxSwitch.pressed = true; // the switch at the + end, whichever way the motor turns
    axis.setTarget(10.0f);
    for (int i = 0; i < 100; ++i)
    {
        (void)axis.tick(kTickMs);
    }
    EXPECT_EQ(gen.total, 0);
}

TEST(StepperLimits, AnAxisWithoutSwitchesIsUnaffected)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(10.0f);
    r.settle();
    EXPECT_FALSE(r.axis.blocked());
    EXPECT_NEAR(r.axis.positionDeg(), 10.0f, 0.05f);
}

TEST(StepperHalt, PulsesHaltedFromOutsideEndTheMoveAndTheAxisRecovers)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(60.0f);
    for (int i = 0; i < 50; ++i)
    {
        ASSERT_TRUE(r.axis.tick(kTickMs));
    }
    ASSERT_TRUE(r.axis.moving());
    r.gen.halt(); // what a limit switch's interrupt does
    const int64_t before = r.gen.total;

    EXPECT_TRUE(r.axis.tick(kTickMs)) << "an expected kind of stop, not a hardware fault";
    EXPECT_TRUE(r.axis.blocked());
    EXPECT_FALSE(r.axis.moving()) << "the move is over";
    EXPECT_FALSE(r.gen.halted()) << "the generator is usable again";
    for (int i = 0; i < 50; ++i)
    {
        (void)r.axis.tick(kTickMs);
    }
    EXPECT_EQ(r.gen.total, before) << "and nothing restarts by itself";

    r.axis.setTarget(-5.0f); // it can be driven away
    r.settle();
    EXPECT_FALSE(r.axis.blocked());
}

TEST(StepperHalt, HaltStopsDeadWhereStopWouldBrake)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(60.0f);
    for (int i = 0; i < 100; ++i)
    {
        (void)r.axis.tick(kTickMs);
    }
    const int64_t before = r.gen.total;
    r.axis.halt();
    for (int i = 0; i < 100; ++i)
    {
        (void)r.axis.tick(kTickMs);
    }
    EXPECT_EQ(r.gen.total, before);
    EXPECT_FALSE(r.axis.blocked()) << "a commanded halt is not a physical obstruction";
}

TEST(StepperHalt, EngagingAfreshClearsTheBlockedMark)
{
    Rig r;
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    r.axis.setTarget(10.0f);
    (void)r.axis.tick(kTickMs);
    r.gen.halt();
    (void)r.axis.tick(kTickMs);
    ASSERT_TRUE(r.axis.blocked());
    ASSERT_TRUE(r.axis.engageAt(0.0f));
    EXPECT_FALSE(r.axis.blocked());
}
