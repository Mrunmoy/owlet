#include "FakeDigitalOutput.hpp"
#include "FakeStepGenerator.hpp"
#include "owlet/core/AxisHoming.hpp"
#include "owlet/stepper/StepperAxis.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>

using ::owlet::StepperAxis;
using ::owlet::core::AxisHoming;
using ::owlet::test::FakeDigitalOutput;
using ::owlet::test::FakeStepGenerator;
using Failure = ::owlet::core::AxisHoming::Failure;
using State = ::owlet::core::AxisHoming::State;

namespace
{

constexpr uint32_t kTickMs = 2;
constexpr float kStepsPerDeg = 9600.0f / 360.0f;

StepperAxis::Config axisConfig(float travelDeg = 170.0f)
{
    StepperAxis::Config c;
    c.stepsPerDeg = kStepsPerDeg;
    c.maxSpeedDegPerSec = 240.0f;
    c.accelDegPerSec2 = 1440.0f;
    c.minDeg = -90.0f;
    c.maxDeg = 90.0f;
    c.travelMinDeg = -travelDeg;
    c.travelMaxDeg = travelDeg;
    return c;
}

// A real StepperAxis on a simulated mechanism. The firmware believes whatever
// it was last told; the mechanism is where the steps actually put it.
struct Mechanism
{
    explicit Mechanism(float startDeg, const StepperAxis::Config& cfg = axisConfig())
        : axis(steps, enable, cfg), m_startDeg(startDeg)
    {
    }

    [[nodiscard]] float physicalDeg() const
    {
        return m_startDeg + static_cast<float>(steps.total) / kStepsPerDeg;
    }

    FakeStepGenerator steps;
    FakeDigitalOutput enable;
    StepperAxis axis;

private:
    float m_startDeg;
};

// A limit switch at a physical angle: pressed at and beyond it.
class Switch final : public ::owlet::hal::IDigitalInput
{
public:
    enum class Side
    {
        Low,
        High,
    };

    Switch(const Mechanism& mechanism, float atDeg, Side side)
        : m_mechanism(mechanism), m_atDeg(atDeg), m_side(side)
    {
    }

    [[nodiscard]] bool active() const override
    {
        if (stuck || missing)
        {
            return stuck;
        }
        return (m_side == Side::Low) ? m_mechanism.physicalDeg() <= m_atDeg
                                     : m_mechanism.physicalDeg() >= m_atDeg;
    }

    bool stuck{false};   // reads pressed whatever the position
    bool missing{false}; // never reads pressed

private:
    const Mechanism& m_mechanism;
    float m_atDeg;
    Side m_side;
};

// Run until homing ends; fails the test if it never does.
void run(Mechanism& m, AxisHoming& homing, int maxTicks = 60000)
{
    int n = 0;
    while (homing.busy() && n++ < maxTicks)
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
        homing.update();
    }
    ASSERT_LT(n, maxTicks) << "homing never finished";
}

} // namespace

// ── One switch: the reference comes from its known angle ────────────────────

TEST(AxisHoming, OneSwitchGivesTheAxisItsTruePosition)
{
    Mechanism m(37.0f); // really at +37°, though the firmware thinks 0
    Switch low(m, -95.0f, Switch::Side::Low);
    AxisHoming::Config cfg;
    cfg.minSwitchDeg = -95.0f;
    AxisHoming homing(m.axis, m.axis, low, nullptr, cfg);

    ASSERT_TRUE(homing.start());
    EXPECT_TRUE(homing.busy());
    run(m, homing);

    EXPECT_EQ(homing.state(), State::Done);
    EXPECT_EQ(homing.failure(), Failure::None);
    EXPECT_NEAR(m.axis.positionDeg(), 0.0f, 0.05f) << "ends at zero";
    EXPECT_NEAR(m.physicalDeg(), 0.0f, 0.1f) << "and zero is where the mechanism's zero really is";
    EXPECT_FLOAT_EQ(m.axis.minDeg(), -90.0f) << "limits are left alone with one switch";
    EXPECT_FLOAT_EQ(m.axis.maxDeg(), 90.0f);
}

TEST(AxisHoming, WorksFromAnywhereIncludingOnTheSwitch)
{
    for (float start : {-97.0f, -95.0f, -60.0f, 0.0f, 88.0f, 160.0f})
    {
        Mechanism m(start);
        Switch low(m, -95.0f, Switch::Side::Low);
        AxisHoming::Config cfg;
        cfg.minSwitchDeg = -95.0f;
        AxisHoming homing(m.axis, m.axis, low, nullptr, cfg);
        ASSERT_TRUE(homing.start());
        run(m, homing);
        EXPECT_EQ(homing.state(), State::Done) << "from " << start;
        EXPECT_NEAR(m.physicalDeg(), 0.0f, 0.1f) << "from " << start;
    }
}

TEST(AxisHoming, TheResultDoesNotDependOnWhereItStarted)
{
    float ends[2] = {0.0f, 0.0f};
    int i = 0;
    for (float start : {-40.0f, 75.0f})
    {
        Mechanism m(start);
        Switch low(m, -95.0f, Switch::Side::Low);
        AxisHoming::Config cfg;
        cfg.minSwitchDeg = -95.0f;
        AxisHoming homing(m.axis, m.axis, low, nullptr, cfg);
        ASSERT_TRUE(homing.start());
        run(m, homing);
        ends[i++] = m.physicalDeg();
    }
    EXPECT_NEAR(ends[0], ends[1], 0.05f) << "repeatable to about a step";
}

TEST(AxisHoming, SeeksGentlyAndApproachesSlower)
{
    Mechanism m(20.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    AxisHoming::Config cfg;
    cfg.minSwitchDeg = -95.0f;
    AxisHoming homing(m.axis, m.axis, low, nullptr, cfg);
    m.axis.setSpeed(200.0f); // the user's setting, far above the seek speed
    ASSERT_TRUE(homing.start());

    const float seekPerTick = cfg.seekSpeedDegPerSec * kStepsPerDeg * (kTickMs / 1000.0f);
    const float approachPerTick = cfg.approachSpeedDegPerSec * kStepsPerDeg * (kTickMs / 1000.0f);
    int n = 0;
    while (homing.busy() && homing.state() != State::Centring && n++ < 60000)
    {
        const std::size_t before = m.steps.bursts.size();
        ASSERT_TRUE(m.axis.tick(kTickMs));
        const int32_t burst =
            (m.steps.bursts.size() > before) ? std::abs(m.steps.bursts.back().steps) : 0;
        const float limit = (homing.state() == State::Approaching) ? approachPerTick : seekPerTick;
        EXPECT_LE(burst, static_cast<int32_t>(std::ceil(limit)) + 1)
            << "in state " << static_cast<int>(homing.state());
        homing.update();
    }
    run(m, homing);
    EXPECT_EQ(homing.state(), State::Done);
    EXPECT_FLOAT_EQ(m.axis.speed(), 200.0f) << "the speed setting is put back";
}

// ── Two switches: the travel is measured, not configured ────────────────────

TEST(AxisHoming, TwoSwitchesSetZeroAtTheMiddleAndTheLimitsJustInside)
{
    Mechanism m(-31.0f);
    Switch low(m, -100.0f, Switch::Side::Low);
    Switch high(m, 80.0f, Switch::Side::High); // not symmetric about the mechanism's own zero
    AxisHoming homing(m.axis, m.axis, low, &high, AxisHoming::Config{});

    ASSERT_TRUE(homing.start());
    run(m, homing);

    EXPECT_EQ(homing.state(), State::Done);
    EXPECT_NEAR(homing.travelDeg(), 180.0f, 0.1f);
    EXPECT_NEAR(m.axis.positionDeg(), 0.0f, 0.05f);
    EXPECT_NEAR(m.physicalDeg(), -10.0f, 0.1f) << "zero is midway between the switches";
    EXPECT_NEAR(m.axis.maxDeg(), 88.0f, 0.15f) << "half the travel, less the margin";
    EXPECT_NEAR(m.axis.minDeg(), -88.0f, 0.15f);

    // The limits now keep the axis off both switches.
    m.axis.setTarget(1000.0f);
    while (m.axis.moving())
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
    }
    EXPECT_FALSE(high.active());
    m.axis.setTarget(-1000.0f);
    while (m.axis.moving())
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
    }
    EXPECT_FALSE(low.active());
}

TEST(AxisHoming, ATravelWiderThanTheAxisAllowsIsRefused)
{
    Mechanism m(0.0f, axisConfig(100.0f)); // the axis accepts at most ±100°
    Switch low(m, -150.0f, Switch::Side::Low);
    Switch high(m, 150.0f, Switch::Side::High);
    AxisHoming homing(m.axis, m.axis, low, &high, AxisHoming::Config{});
    ASSERT_TRUE(homing.start());
    run(m, homing);
    EXPECT_EQ(homing.state(), State::Failed);
    EXPECT_EQ(homing.failure(), Failure::OutOfTravel);
    EXPECT_FALSE(m.axis.moving());
}

// ── Failures end safely ─────────────────────────────────────────────────────

TEST(AxisHoming, GivesUpWhenThereIsNoSwitchToFind)
{
    Mechanism m(0.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    low.missing = true; // unplugged
    AxisHoming homing(m.axis, m.axis, low, nullptr, AxisHoming::Config{});
    m.axis.setSpeed(150.0f);
    ASSERT_TRUE(homing.start());
    run(m, homing);
    EXPECT_EQ(homing.state(), State::Failed);
    EXPECT_EQ(homing.failure(), Failure::SwitchNotFound);
    EXPECT_FALSE(m.axis.moving());
    EXPECT_FLOAT_EQ(m.axis.speed(), 150.0f);
    EXPECT_LE(std::fabs(m.physicalDeg()), 365.0f) << "looked no further than the search span";
}

TEST(AxisHoming, ASwitchThatNeverReleasesIsReported)
{
    Mechanism m(0.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    low.stuck = true; // shorted, or wired the wrong way round
    AxisHoming homing(m.axis, m.axis, low, nullptr, AxisHoming::Config{});
    ASSERT_TRUE(homing.start());
    run(m, homing);
    EXPECT_EQ(homing.failure(), Failure::SwitchStuck);
    EXPECT_LE(std::fabs(m.physicalDeg()), 5.0f) << "it only backed off; it did not drive anywhere";
}

TEST(AxisHoming, AbortBrakesAndReports)
{
    Mechanism m(60.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    AxisHoming homing(m.axis, m.axis, low, nullptr, AxisHoming::Config{});
    ASSERT_TRUE(homing.start());
    for (int i = 0; i < 500; ++i)
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
        homing.update();
    }
    ASSERT_EQ(homing.state(), State::Seeking);
    homing.abort();
    EXPECT_EQ(homing.failure(), Failure::Aborted);
    EXPECT_FALSE(homing.busy());
    while (m.axis.moving())
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
    }
    homing.update(); // no effect once ended
    EXPECT_EQ(homing.state(), State::Failed);
}

TEST(AxisHoming, LosingPowerMidwayIsAnAbort)
{
    Mechanism m(60.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    AxisHoming homing(m.axis, m.axis, low, nullptr, AxisHoming::Config{});
    ASSERT_TRUE(homing.start());
    for (int i = 0; i < 200; ++i)
    {
        ASSERT_TRUE(m.axis.tick(kTickMs));
        homing.update();
    }
    ASSERT_TRUE(m.axis.release()); // someone sent DISABLE
    homing.update();
    EXPECT_EQ(homing.failure(), Failure::Aborted);
}

TEST(AxisHoming, CannotBeStartedTwice)
{
    Mechanism m(10.0f);
    Switch low(m, -95.0f, Switch::Side::Low);
    AxisHoming homing(m.axis, m.axis, low, nullptr, AxisHoming::Config{});
    EXPECT_EQ(homing.state(), State::Idle);
    ASSERT_TRUE(homing.start());
    EXPECT_FALSE(homing.start());
    run(m, homing);
    EXPECT_TRUE(homing.start()) << "but it can be run again afterwards";
}

// ── The whole head: switches, interrupts, policy and homing together ────────

#include "RecordingEventSink.hpp"
#include "owlet/core/HeadHoming.hpp"
#include "owlet/core/PanTilt.hpp"

#include <vector>

namespace
{

using ::owlet::core::EventId;
using ::owlet::core::HeadHoming;
using ::owlet::core::PanTilt;
using Result = ::owlet::core::IPanTilt::Result;

// A switch that reads the mechanism through the step count alone, so it can
// exist before the axis it guards.
class EndSwitch final : public ::owlet::hal::IDigitalInput
{
public:
    EndSwitch(const FakeStepGenerator& steps, float startDeg, float atDeg, bool high)
        : m_steps(steps), m_startDeg(startDeg), m_atDeg(atDeg), m_high(high)
    {
    }
    [[nodiscard]] bool active() const override
    {
        const float deg = m_startDeg + static_cast<float>(m_steps.total) / kStepsPerDeg;
        return m_high ? deg >= m_atDeg : deg <= m_atDeg;
    }

private:
    const FakeStepGenerator& m_steps;
    float m_startDeg;
    float m_atDeg;
    bool m_high;
};

// One axis as it will be built: the switches guard the axis, home it, and
// (through pressEdges) halt its pulses the way their interrupt will.
struct GuardedAxis
{
    GuardedAxis(float startDeg, float lowAt, float highAt)
        : start(startDeg), low(steps, startDeg, lowAt, false), high(steps, startDeg, highAt, true),
          axis(steps, enable, axisConfig(), StepperAxis::LimitSwitches{&low, &high}),
          homing(axis, axis, low, &high, AxisHoming::Config{})
    {
    }

    [[nodiscard]] float physicalDeg() const
    {
        return start + static_cast<float>(steps.total) / kStepsPerDeg;
    }

    // Stands in for the edge interrupt: a switch that has just closed halts
    // the pulses and reports it.
    void pressEdges(::owlet::core::IEventSink& sink, uint8_t axisIndex)
    {
        const bool lowNow = low.active();
        const bool highNow = high.active();
        if ((lowNow && !m_lowWas) || (highNow && !m_highWas))
        {
            steps.halt();
            (void)sink.post(EventId::LimitHit, axisIndex, (highNow && !m_highWas) ? 1 : 0);
        }
        m_lowWas = lowNow;
        m_highWas = highNow;
    }

    float start;
    FakeStepGenerator steps;
    FakeDigitalOutput enable;
    EndSwitch low;
    EndSwitch high;
    StepperAxis axis;
    AxisHoming homing;

private:
    bool m_lowWas{false};
    bool m_highWas{false};
};

struct HeadRig
{
    // Pan really sits at +80° and tilt at -20°, though both believe 0.
    GuardedAxis pan{80.0f, -100.0f, 100.0f};
    GuardedAxis tilt{-20.0f, -70.0f, 70.0f};
    ::owlet::test::RecordingEventSink events;
    HeadHoming homing{pan.homing, tilt.homing};
    PanTilt head{pan.axis, tilt.axis, &events, &homing};

    void run(int ticks)
    {
        for (int i = 0; i < ticks; ++i)
        {
            (void)head.tick(kTickMs);
            pan.pressEdges(events, 0);
            tilt.pressEdges(events, 1);
        }
    }

    [[nodiscard]] int count(EventId id) const
    {
        int n = 0;
        for (const ::owlet::core::Event& e : events.events)
        {
            n += (e.id == id) ? 1 : 0;
        }
        return n;
    }
};

} // namespace

TEST(HeadHoming, RunningIntoASwitchStopsTheHeadAndItRefusesToMove)
{
    HeadRig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(60.0f, 0.0f), Result::Ok); // believed safe; really heading for +140°
    r.run(3000);

    EXPECT_LT(r.pan.physicalDeg(), 100.6f) << "stopped at the switch, within a burst";
    EXPECT_EQ(r.count(EventId::LimitHit), 1);
    EXPECT_EQ(r.count(EventId::PositionLost), 1);
    EXPECT_EQ(r.count(EventId::Arrived), 0) << "it did not arrive, and does not say it did";
    EXPECT_FALSE(r.head.positionTrusted());
    EXPECT_EQ(r.head.move(0.0f, 0.0f), Result::Untrusted);
    const float rested = r.pan.physicalDeg();
    r.run(500);
    EXPECT_FLOAT_EQ(r.pan.physicalDeg(), rested);
}

TEST(HeadHoming, SeekingFindsBothAxesAndMakesThePositionTrustworthy)
{
    HeadRig r;
    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    r.run(60000);

    ASSERT_EQ(r.count(EventId::Homed), 1) << "failed on axis " << int(r.homing.failedAxis()) << " code "
                                          << r.homing.failureCode();
    EXPECT_EQ(r.count(EventId::PositionLost), 0) << "switches met while homing are expected";
    EXPECT_TRUE(r.head.positionTrusted());
    EXPECT_NEAR(r.pan.physicalDeg(), 0.0f, 0.2f) << "zero is midway between the switches";
    EXPECT_NEAR(r.tilt.physicalDeg(), 0.0f, 0.2f);
    EXPECT_NEAR(r.pan.axis.maxDeg(), 98.0f, 0.3f) << "limits sit just inside the switches";
    EXPECT_NEAR(r.tilt.axis.minDeg(), -68.0f, 0.3f);

    ASSERT_EQ(r.head.move(90.0f, -60.0f), Result::Ok);
    r.run(3000);
    EXPECT_NEAR(r.pan.physicalDeg(), 90.0f, 0.2f);
    EXPECT_NEAR(r.tilt.physicalDeg(), -60.0f, 0.2f);
    EXPECT_EQ(r.count(EventId::Arrived), 1);
}

TEST(HeadHoming, ALostHeadIsRecoveredBySeeking)
{
    HeadRig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(60.0f, 0.0f), Result::Ok);
    r.run(3000);
    ASSERT_FALSE(r.head.positionTrusted());

    ASSERT_EQ(r.head.seekHome(), Result::Ok);
    r.run(60000);
    EXPECT_EQ(r.count(EventId::Homed), 1);
    EXPECT_TRUE(r.head.positionTrusted());
    EXPECT_NEAR(r.pan.physicalDeg(), 0.0f, 0.2f);
}

TEST(HeadHoming, AMissingSwitchFailsThatAxisAndNamesIt)
{
    HeadRig r;
    // Pan's switches are out of reach of the search.
    GuardedAxis farPan{0.0f, -5000.0f, 5000.0f};
    HeadHoming homing{farPan.homing, r.tilt.homing};
    ::owlet::test::RecordingEventSink events;
    PanTilt head{farPan.axis, r.tilt.axis, &events, &homing};
    ASSERT_EQ(head.seekHome(), Result::Ok);
    for (int i = 0; i < 120000 && homing.busy(); ++i)
    {
        (void)head.tick(kTickMs);
        r.tilt.pressEdges(events, 1);
    }
    ASSERT_FALSE(homing.busy());
    ASSERT_FALSE(events.events.empty());
    const ::owlet::core::Event& last = events.events.back();
    EXPECT_EQ(last.id, EventId::HomingFailed);
    EXPECT_EQ(last.axis, 0) << "tilt homed; pan is the one that failed";
    EXPECT_EQ(last.arg, static_cast<uint16_t>(Failure::SwitchNotFound));
    EXPECT_FALSE(head.positionTrusted());
}
