#include "FakeAxis.hpp"
#include "FakeDigitalOutput.hpp"
#include "owlet/core/ActivityIndicator.hpp"
#include "owlet/core/PanTilt.hpp"

#include <gtest/gtest.h>

using ::owlet::core::ActivityIndicator;
using ::owlet::core::IPanTilt;
using ::owlet::core::PanTilt;
using ::owlet::test::FakeAxis;
using ::owlet::test::FakeDigitalOutput;

TEST(ActivityIndicator, LitOnlyWhileTheHeadMovesAndWritesOnChange)
{
    FakeAxis pan;
    FakeAxis tilt;
    PanTilt head(pan, tilt);
    FakeDigitalOutput led;
    ActivityIndicator indicator(head, head, led);

    EXPECT_TRUE(indicator.tick(2).busOk);
    EXPECT_FALSE(led.on);
    EXPECT_EQ(led.writes, 0) << "no change, no write";

    ASSERT_EQ(head.enable(), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setWatchdog(0), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setVelocity(10.0f, 0.0f), IPanTilt::Result::Ok);
    (void)indicator.tick(2);
    (void)indicator.tick(2);
    EXPECT_TRUE(led.on);
    EXPECT_EQ(led.writes, 1);
    EXPECT_EQ(pan.ticks, 3) << "the wrapped motion layer is ticked every time";

    ASSERT_EQ(head.stop(), IPanTilt::Result::Ok);
    (void)indicator.tick(2);
    EXPECT_FALSE(led.on);
    EXPECT_EQ(led.writes, 2);
}

TEST(ActivityIndicator, PassesTheWatchdogStatusThrough)
{
    FakeAxis pan;
    FakeAxis tilt;
    PanTilt head(pan, tilt);
    FakeDigitalOutput led;
    ActivityIndicator indicator(head, head, led);
    ASSERT_EQ(head.enable(), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setWatchdog(50), IPanTilt::Result::Ok);
    ASSERT_EQ(head.setVelocity(10.0f, 0.0f), IPanTilt::Result::Ok);
    EXPECT_TRUE(indicator.tick(60).watchdogTripped);
}
