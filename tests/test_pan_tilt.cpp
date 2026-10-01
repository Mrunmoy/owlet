#include "FakeServoChannel.hpp"
#include "owlet/core/PanTilt.hpp"

#include <gtest/gtest.h>

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
    EXPECT_FLOAT_EQ(r.head.panDeg(), 45.0f);
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
    EXPECT_FLOAT_EQ(r.head.panTargetDeg(), 60.0f);
}

TEST(PanTilt, ReEnableAfterMidMoveDisableUsesPositionNotTarget)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(200));  // pan at 20°
    ASSERT_EQ(r.head.disable(), Result::Ok);
    EXPECT_TRUE(r.panCh.idle);
    ASSERT_EQ(r.head.enable(), Result::Ok);
    EXPECT_NEAR(r.head.panDeg(), 20.0f, 1e-3f);
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
    ASSERT_TRUE(r.head.tick(100));
    EXPECT_NEAR(r.head.panDeg(), 10.0f, 1e-3f);
    EXPECT_NEAR(r.head.tiltDeg(), -10.0f, 1e-3f);  // tilt already arrived
    EXPECT_EQ(r.head.state(), State::Moving);
    ASSERT_TRUE(r.head.tick(200));
    EXPECT_EQ(r.head.state(), State::Holding);
}

TEST(PanTilt, NudgeIsRelativeToTargetAndClamps)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), Result::Ok);
    ASSERT_EQ(r.head.nudge(5.0f, -3.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.panTargetDeg(), 85.0f);
    EXPECT_FLOAT_EQ(r.head.tiltTargetDeg(), -3.0f);
    ASSERT_EQ(r.head.nudge(50.0f, 0.0f), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.panTargetDeg(), 90.0f);
}

TEST(PanTilt, HomeTargetsCenter)
{
    Rig r;
    ASSERT_EQ(r.head.enableAt(40.0f, 40.0f), Result::Ok);
    ASSERT_EQ(r.head.home(), Result::Ok);
    EXPECT_FLOAT_EQ(r.head.panTargetDeg(), 0.0f);
    EXPECT_FLOAT_EQ(r.head.tiltTargetDeg(), 0.0f);
}

TEST(PanTilt, StopFreezesWhereItIs)
{
    Rig r;
    ASSERT_EQ(r.head.enable(), Result::Ok);
    ASSERT_EQ(r.head.setSpeed(100.0f), Result::Ok);
    ASSERT_EQ(r.head.move(50.0f, 50.0f), Result::Ok);
    ASSERT_TRUE(r.head.tick(150));
    ASSERT_EQ(r.head.stop(), Result::Ok);
    EXPECT_EQ(r.head.state(), State::Holding);
    EXPECT_NEAR(r.head.panTargetDeg(), 15.0f, 1e-3f);
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
