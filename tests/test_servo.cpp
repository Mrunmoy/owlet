#include "FakeServoChannel.hpp"
#include "owlet/servo/Servo.hpp"

#include <gtest/gtest.h>

using ::owlet::Servo;
using ::owlet::test::FakeServoChannel;

namespace
{

Servo::Calibration defaultCal()
{
    return Servo::Calibration{};
}

} // namespace

// ── Calibration / mapping ───────────────────────────────────────────────────

TEST(Servo, DefaultCalibrationIsPlusMinus90Over500To2500)
{
    const Servo::Calibration cal;
    EXPECT_EQ(cal.minPulseUs, 500);
    EXPECT_EQ(cal.centerPulseUs, 1500);
    EXPECT_EQ(cal.maxPulseUs, 2500);
    EXPECT_FLOAT_EQ(cal.minDeg, -90.0f);
    EXPECT_FLOAT_EQ(cal.maxDeg, 90.0f);
}

TEST(Servo, MapsCenterAndEndpoints)
{
    FakeServoChannel ch;
    const Servo s(ch, defaultCal());
    EXPECT_EQ(s.angleToPulse(0.0f), 1500);
    EXPECT_EQ(s.angleToPulse(-90.0f), 500);
    EXPECT_EQ(s.angleToPulse(90.0f), 2500);
    EXPECT_EQ(s.angleToPulse(45.0f), 2000);
    EXPECT_EQ(s.angleToPulse(-22.5f), 1250);
}

TEST(Servo, MappingIsPiecewiseThroughCenter)
{
    // Asymmetric servo: center is not the midpoint of the pulse range.
    Servo::Calibration cal;
    cal.minPulseUs    = 600;
    cal.centerPulseUs = 1400;
    cal.maxPulseUs    = 2400;
    FakeServoChannel ch;
    const Servo s(ch, cal);
    EXPECT_EQ(s.angleToPulse(0.0f), 1400);
    EXPECT_EQ(s.angleToPulse(-45.0f), 1000);  // halfway 1400 → 600
    EXPECT_EQ(s.angleToPulse(45.0f), 1900);   // halfway 1400 → 2400
}

TEST(Servo, ReversedAxisSwapsPulseDirection)
{
    Servo::Calibration cal;
    cal.minPulseUs = 2500;
    cal.maxPulseUs = 500;
    FakeServoChannel ch;
    const Servo s(ch, cal);
    EXPECT_EQ(s.angleToPulse(-90.0f), 2500);
    EXPECT_EQ(s.angleToPulse(90.0f), 500);
    EXPECT_EQ(s.angleToPulse(30.0f), 1167);
}

TEST(Servo, ClampsTargetToCalibratedLimits)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    s.setTarget(120.0f);
    EXPECT_FLOAT_EQ(s.targetDeg(), 90.0f);
    s.setTarget(-200.0f);
    EXPECT_FLOAT_EQ(s.targetDeg(), -90.0f);
}

TEST(Servo, NarrowerSoftLimitsAreHonoured)
{
    Servo::Calibration cal;
    cal.minDeg = -60.0f;
    cal.maxDeg = 45.0f;
    FakeServoChannel ch;
    Servo s(ch, cal);
    s.setTarget(80.0f);
    EXPECT_FLOAT_EQ(s.targetDeg(), 45.0f);
    // Mapping still uses the ±90° pulse span: soft limits only restrict range.
    EXPECT_EQ(s.angleToPulse(45.0f), 2000);
}

// ── Engage / release ────────────────────────────────────────────────────────

TEST(Servo, StartsReleasedAndWritesNothing)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    EXPECT_FALSE(s.engaged());
    EXPECT_TRUE(s.tick(20));
    EXPECT_TRUE(ch.writes.empty());
}

TEST(Servo, EngageWritesCurrentPositionImmediately)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(30.0f));
    EXPECT_TRUE(s.engaged());
    ASSERT_EQ(ch.writes.size(), 1u);
    EXPECT_EQ(ch.lastPulseUs, s.angleToPulse(30.0f));
    EXPECT_FLOAT_EQ(s.positionDeg(), 30.0f);
    EXPECT_FLOAT_EQ(s.targetDeg(), 30.0f);
    EXPECT_FALSE(s.moving());
}

TEST(Servo, ReleaseIdlesChannelAndKeepsLastPosition)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(10.0f));
    ASSERT_TRUE(s.release());
    EXPECT_FALSE(s.engaged());
    EXPECT_TRUE(ch.idle);
    EXPECT_FLOAT_EQ(s.positionDeg(), 10.0f);
}

TEST(Servo, EngageFailurePropagatesAndStaysReleased)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ch.failNext = true;
    EXPECT_FALSE(s.engageAt(0.0f));
    EXPECT_FALSE(s.engaged());
}

// ── Slew ────────────────────────────────────────────────────────────────────

TEST(Servo, SlewsAtConfiguredDegreesPerSecond)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(0.0f));
    s.setSpeed(90.0f);
    s.setTarget(45.0f);
    EXPECT_TRUE(s.moving());

    ASSERT_TRUE(s.tick(100));  // 0.1 s × 90 deg/s = 9°
    EXPECT_NEAR(s.positionDeg(), 9.0f, 1e-4f);
    ASSERT_TRUE(s.tick(400));  // +36° → 45°, reached
    EXPECT_NEAR(s.positionDeg(), 45.0f, 1e-4f);
    EXPECT_FALSE(s.moving());
}

TEST(Servo, DoesNotOvershootTarget)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(0.0f));
    s.setSpeed(500.0f);
    s.setTarget(-5.0f);
    ASSERT_TRUE(s.tick(1000));
    EXPECT_FLOAT_EQ(s.positionDeg(), -5.0f);
    EXPECT_EQ(ch.lastPulseUs, s.angleToPulse(-5.0f));
}

TEST(Servo, WritesOnlyWhenPulseChanges)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(0.0f));
    const std::size_t afterEngage = ch.writes.size();
    ASSERT_TRUE(s.tick(20));
    ASSERT_TRUE(s.tick(20));
    EXPECT_EQ(ch.writes.size(), afterEngage);
}

TEST(Servo, ReleasedServoDoesNotSlew)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    s.setTarget(45.0f);
    ASSERT_TRUE(s.tick(1000));
    EXPECT_FLOAT_EQ(s.positionDeg(), 0.0f);
    EXPECT_TRUE(ch.writes.empty());
}

TEST(Servo, StopFreezesTargetAtCurrentPosition)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(0.0f));
    s.setSpeed(100.0f);
    s.setTarget(90.0f);
    ASSERT_TRUE(s.tick(250));  // 25°
    s.stop();
    EXPECT_FALSE(s.moving());
    EXPECT_NEAR(s.targetDeg(), 25.0f, 1e-4f);
}

TEST(Servo, SpeedIsClampedToSupportedRange)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    s.setSpeed(0.0f);
    EXPECT_FLOAT_EQ(s.speed(), Servo::kMinSpeedDegPerSec);
    s.setSpeed(10'000.0f);
    EXPECT_FLOAT_EQ(s.speed(), Servo::kMaxSpeedDegPerSec);
}

TEST(Servo, TickPropagatesBusErrorAndRetriesNextTick)
{
    FakeServoChannel ch;
    Servo s(ch, defaultCal());
    ASSERT_TRUE(s.engageAt(0.0f));
    s.setSpeed(100.0f);
    s.setTarget(10.0f);
    ch.failNext = true;
    EXPECT_FALSE(s.tick(50));
    // Position estimate must not advance past what hardware actually received.
    EXPECT_FLOAT_EQ(s.positionDeg(), 0.0f);
    EXPECT_TRUE(s.tick(50));
    EXPECT_GT(s.positionDeg(), 0.0f);
}
