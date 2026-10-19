#include "FakeCalibrationStore.hpp"
#include "ReferenceStepperHead.hpp"
#include "owlet/core/CalibrationCodec.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/stepper/StepperAxis.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <string>

using ::owlet::StepperAxis;
using ::owlet::hal::AxisCalibration;
using ::owlet::core::Axis;
using ::owlet::core::CalibrationCodec;
using ::owlet::core::CalibrationRecord;
using ::owlet::core::Calibrator;
using ::owlet::core::ICalibration;
using ::owlet::core::PanTilt;
using ::owlet::test::FakeCalibrationStore;
using ::owlet::test::ReferenceAxes;
using CalResult = ::owlet::core::ICalibration::Result;
using Source    = ::owlet::core::ICalibration::Source;
using HeadResult = ::owlet::core::IPanTilt::Result;

namespace
{

AxisCalibration cal(uint16_t driveMin, uint16_t driveCenter, uint16_t driveMax, float minDeg,
                    float maxDeg)
{
    return AxisCalibration{driveMin, driveCenter, driveMax, minDeg, maxDeg};
}

// A stepper's calibration is its soft limits; its drive numbers are always zero.
AxisCalibration limits(float minDeg, float maxDeg)
{
    return cal(0, 0, 0, minDeg, maxDeg);
}

CalibrationRecord factory()
{
    return CalibrationRecord{limits(-90, 90), limits(-60, 60)};
}

bool same(const AxisCalibration& a, const AxisCalibration& b)
{
    return a.driveMin == b.driveMin && a.driveCenter == b.driveCenter &&
           a.driveMax == b.driveMax && a.minDeg == b.minDeg && a.maxDeg == b.maxDeg;
}

struct Rig : ReferenceAxes
{
    PanTilt              head{pan, tilt};
    FakeCalibrationStore store;
    Calibrator           calib{pan, tilt, head, store, factory()};
};

} // namespace

// ── CalibrationCodec ────────────────────────────────────────────────────────

TEST(CalibrationCodec, Crc32KnownAnswer)
{
    const char* s = "123456789";
    EXPECT_EQ(CalibrationCodec::crc32(reinterpret_cast<const uint8_t*>(s), 9), 0xCBF43926u);
}

TEST(CalibrationCodec, RoundTrips)
{
    // The record keeps all three drive numbers for any axis type that has them.
    const CalibrationRecord in{cal(550, 1480, 2420, -85, 90), limits(-45.5f, 135)};
    const auto              bytes = CalibrationCodec::encode(in);
    CalibrationRecord       out{};
    ASSERT_TRUE(CalibrationCodec::decode(bytes.data(), bytes.size(), out));
    EXPECT_TRUE(same(in.pan, out.pan));
    EXPECT_TRUE(same(in.tilt, out.tilt));
}

TEST(CalibrationCodec, RejectsCorruption)
{
    const auto bytes = CalibrationCodec::encode(factory());
    for (std::size_t i = 0; i < bytes.size(); ++i)
    {
        auto bad = bytes;
        bad[i] ^= 0x01;
        CalibrationRecord out{};
        EXPECT_FALSE(CalibrationCodec::decode(bad.data(), bad.size(), out)) << "byte " << i;
    }
}

TEST(CalibrationCodec, RejectsWrongSize)
{
    const auto        bytes = CalibrationCodec::encode(factory());
    CalibrationRecord out{};
    EXPECT_FALSE(CalibrationCodec::decode(bytes.data(), bytes.size() - 1, out));
    EXPECT_FALSE(CalibrationCodec::decode(nullptr, 0, out));
}

// ── Capabilities ────────────────────────────────────────────────────────────

TEST(PanTiltCapabilities, ReportsMotionRanges)
{
    Rig                                     r;
    const ::owlet::core::IPanTilt::Capabilities c = r.head.capabilities();
    EXPECT_FLOAT_EQ(c.speedMin, 1.0f);
    EXPECT_FLOAT_EQ(c.speedMax, 240.0f);
    EXPECT_FLOAT_EQ(c.velocityMax, 240.0f);
    EXPECT_EQ(c.watchdogMinMs, 50u);
    EXPECT_EQ(c.watchdogMaxMs, 5000u);
}

// ── Calibrator ──────────────────────────────────────────────────────────────

TEST(Calibrator, BootsWithFactoryValuesWhenNothingIsSaved)
{
    Rig r;
    r.calib.begin();
    EXPECT_EQ(r.calib.source(), Source::Factory);
    EXPECT_TRUE(same(r.calib.get(Axis::Tilt), factory().tilt));
}

TEST(Calibrator, BootsWithTheSavedRecord)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{limits(-85, 90), limits(-45, 120)};
    r.calib.begin();
    EXPECT_EQ(r.calib.source(), Source::Saved);
    EXPECT_TRUE(same(r.calib.get(Axis::Tilt), r.store.record.tilt));
    EXPECT_FLOAT_EQ(r.head.limits().panMin, -85.0f);
    EXPECT_FLOAT_EQ(r.head.limits().tiltMax, 120.0f);
}

TEST(Calibrator, IgnoresASavedRecordThatFailsValidation)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{cal(500, 1500, 2500, -90, 90), factory().tilt};
    r.calib.begin();
    EXPECT_EQ(r.calib.source(), Source::Factory);
    EXPECT_TRUE(same(r.calib.get(Axis::Pan), factory().pan));
}

TEST(Calibrator, LimitsCanChangeWhileEnabledAndPullTheTargetIn)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    ASSERT_EQ(r.head.move(80.0f, 0.0f), HeadResult::Ok);
    EXPECT_EQ(r.calib.set(Axis::Pan, limits(-85, 60)), CalResult::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 60.0f);
    EXPECT_FLOAT_EQ(r.head.limits().panMin, -85.0f);
    EXPECT_EQ(r.calib.source(), Source::Modified);
}

TEST(Calibrator, RejectsInvalidCalibration)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    Rig         r;
    r.calib.begin();
    const AxisCalibration bad[] = {
        cal(500, 1500, 2500, -90, 90), // drive numbers another motor might use
        cal(1, 0, 0, -90, 90),         // any drive number at all
        cal(0, 1, 0, -90, 90),
        cal(0, 0, 1, -90, 90),
        limits(-181, 90),              // beyond the pan travel of ±180
        limits(-90, 181),
        limits(10, 90),                // min limit above centre
        limits(-90, -10),              // max limit below centre
        limits(0, 0),                  // no room to move
        limits(nan, 90),               // not a number
        limits(-90, inf),
    };
    for (const AxisCalibration& c : bad)
    {
        EXPECT_EQ(r.calib.set(Axis::Pan, c), CalResult::Range)
            << c.driveMin << "/" << c.driveCenter << "/" << c.driveMax << " " << c.minDeg
            << ".." << c.maxDeg;
    }
    EXPECT_TRUE(same(r.calib.get(Axis::Pan), factory().pan));
    EXPECT_EQ(r.calib.source(), Source::Factory);
}

TEST(Calibrator, EachAxisIsLimitedByItsOwnTravel)
{
    Rig r;
    r.calib.begin();
    EXPECT_EQ(r.calib.set(Axis::Pan, limits(-180, 180)), CalResult::Ok);
    EXPECT_EQ(r.calib.set(Axis::Tilt, limits(-135, 135)), CalResult::Ok);
    EXPECT_EQ(r.calib.set(Axis::Tilt, limits(-136, 135)), CalResult::Range)
        << "the tilt cradle has less room than the pan base";
}

TEST(Calibrator, LimitChangesNeverNeedTheHeadDisabled)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_EQ(r.calib.set(Axis::Pan, limits(-120, 120)), CalResult::Ok)
        << "limits never move the shaft by themselves";
    EXPECT_EQ(r.calib.set(Axis::Tilt, limits(-30, 30)), CalResult::Ok);
    EXPECT_FALSE(r.anyStepped());
}

TEST(Calibrator, SaveStoresBothAxes)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Tilt, limits(-45, 120)), CalResult::Ok);
    EXPECT_EQ(r.calib.save(), CalResult::Ok);
    EXPECT_EQ(r.calib.source(), Source::Saved);
    ASSERT_TRUE(r.store.hasRecord);
    EXPECT_TRUE(same(r.store.record.tilt, limits(-45, 120)));
    EXPECT_TRUE(same(r.store.record.pan, factory().pan));
}

TEST(Calibrator, SaveFailureIsReported)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Tilt, limits(-45, 120)), CalResult::Ok);
    r.store.failSave = true;
    EXPECT_EQ(r.calib.save(), CalResult::Storage);
    EXPECT_EQ(r.calib.source(), Source::Modified);
}

TEST(Calibrator, ResetRestoresFactoryAndErasesTheSavedCopy)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{factory().pan, limits(-45, 120)};
    r.calib.begin();
    EXPECT_EQ(r.calib.reset(), CalResult::Ok);
    EXPECT_EQ(r.calib.source(), Source::Factory);
    EXPECT_TRUE(same(r.calib.get(Axis::Tilt), factory().tilt));
    EXPECT_FALSE(r.store.hasRecord);
}

TEST(Calibrator, ResetIsRefusedWhileEnabled)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{factory().pan, limits(-45, 120)};
    r.calib.begin();
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_EQ(r.calib.reset(), CalResult::Enabled);
    EXPECT_EQ(r.calib.source(), Source::Saved);
}

TEST(Calibrator, ResetWithFailedEraseChangesNothing)  // review C5
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{limits(-85, 90), factory().tilt};
    r.calib.begin();
    r.store.failErase = true;
    EXPECT_EQ(r.calib.reset(), CalResult::Storage);
    // RAM, flash and the reported source all still agree: the saved record.
    EXPECT_EQ(r.calib.source(), Source::Saved);
    EXPECT_FLOAT_EQ(r.calib.get(Axis::Pan).minDeg, -85.0f);
}

// ── Review findings (calibration round, 2026-10-01) ─────────────────────────

TEST(CalibratorReview, InvalidFactoryRecordFallsBackToSafeDefaults)  // C4
{
    ReferenceAxes        axes;
    PanTilt              head{axes.pan, axes.tilt};
    FakeCalibrationStore store;
    Calibrator calib{axes.pan, axes.tilt, head, store,
                     CalibrationRecord{cal(1500, 2600, 1550, 0, 0), factory().tilt}};
    calib.begin();
    EXPECT_FALSE(calib.factoryWasValid());
    EXPECT_TRUE(axes.pan.acceptsCalibration(calib.get(Axis::Pan)));
    EXPECT_TRUE(same(calib.get(Axis::Pan), axes.pan.defaultCalibration()));
    EXPECT_EQ(calib.reset(), CalResult::Ok);  // reset re-applies the *safe* factory
    EXPECT_TRUE(axes.pan.acceptsCalibration(calib.get(Axis::Pan)));
}

TEST(CalibratorReview, NarrowingLimitsInVelocityModeNeverJumps)  // C1
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.setWatchdog(0), HeadResult::Ok);
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.setVelocity(-10.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, limits(-30, 30)), CalResult::Ok);
    for (int i = 0; i < 300; ++i)
    {
        const float before = r.head.position().pan;
        ASSERT_TRUE(r.head.tick(10).busOk);
        // speed() is the default 90°/s: never more than that per tick
        EXPECT_LE(before - r.head.position().pan, 90.0f * 0.010f + 1e-3f) << "tick " << i;
    }
    EXPECT_FLOAT_EQ(r.head.position().pan, 30.0f);  // travelled inside, not snapped
}

TEST(CalibratorReview, NarrowingLimitsWhilePinnedSlewsInAndLaterVelocityIsSmooth)  // C1
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.setWatchdog(0), HeadResult::Ok);
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, limits(-30, 30)), CalResult::Ok);
    for (int i = 0; i < 100; ++i)
    {
        ASSERT_TRUE(r.head.tick(10).busOk);
    }
    EXPECT_FLOAT_EQ(r.head.position().pan, 30.0f);
    ASSERT_EQ(r.head.setVelocity(-1.0f, 0.0f), HeadResult::Ok);
    for (int i = 0; i < 100; ++i)
    {
        const float before = r.head.position().pan;
        ASSERT_TRUE(r.head.tick(10).busOk);
        // No catching up on a backlog: one step at most per tick at 1°/s.
        EXPECT_LE(before - r.head.position().pan, ::owlet::test::referenceStepDeg() + 1e-4f)
            << "tick " << i;
    }
    EXPECT_NEAR(r.head.position().pan, 29.0f, 2.0f * ::owlet::test::referenceStepDeg());
}

TEST(CalibratorReview, BareEnableIsRefusedWhenTheShaftIsOutsideNewLimits)  // C2
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enableAt(80.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, limits(-90, 60)), CalResult::Ok);
    EXPECT_EQ(r.head.enable(), HeadResult::Range);  // clamping would misname 80° as 60°
    EXPECT_FALSE(r.anyPowered());
    EXPECT_EQ(r.head.state(), ::owlet::core::IPanTilt::State::Disabled);
    EXPECT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);  // explicit pose is the user's call
    EXPECT_FALSE(r.anyStepped());
}

TEST(CalibratorReview, LimitsAreStoredAtTenthOfADegree)  // C7
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Pan, limits(-85.06f, 89.94f)), CalResult::Ok);
    EXPECT_FLOAT_EQ(r.calib.get(Axis::Pan).minDeg, -85.1f);
    EXPECT_FLOAT_EQ(r.calib.get(Axis::Pan).maxDeg, 89.9f);
}

TEST(CalibratorReview, RoundingNoiseAtALimitDoesNotBlockEnable)  // N8
{
    Rig r;
    r.calib.begin();
    // 45.4° is not a whole number of steps: the nearest lies just beyond it.
    ASSERT_EQ(r.calib.set(Axis::Pan, limits(-45.4f, 45.4f)), CalResult::Ok);
    ASSERT_EQ(r.head.enableAt(45.4f, 0.0f), HeadResult::Ok);
    ASSERT_GT(r.head.position().pan, 45.4f);
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    EXPECT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_FALSE(r.anyStepped());
}
