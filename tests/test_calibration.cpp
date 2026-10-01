#include "FakeCalibrationStore.hpp"
#include "FakeServoChannel.hpp"
#include "owlet/core/CalibrationCodec.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/PanTilt.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <string>

using ::owlet::Servo;
using ::owlet::core::Axis;
using ::owlet::core::CalibrationCodec;
using ::owlet::core::CalibrationRecord;
using ::owlet::core::Calibrator;
using ::owlet::core::ICalibration;
using ::owlet::core::PanTilt;
using ::owlet::test::FakeCalibrationStore;
using ::owlet::test::FakeServoChannel;
using CalResult = ::owlet::core::ICalibration::Result;
using Source    = ::owlet::core::ICalibration::Source;
using HeadResult = ::owlet::core::IPanTilt::Result;

namespace
{

Servo::Calibration cal(uint16_t minUs, uint16_t centerUs, uint16_t maxUs, float minDeg, float maxDeg)
{
    Servo::Calibration c;
    c.minPulseUs    = minUs;
    c.centerPulseUs = centerUs;
    c.maxPulseUs    = maxUs;
    c.minDeg        = minDeg;
    c.maxDeg        = maxDeg;
    return c;
}

CalibrationRecord factory()
{
    return CalibrationRecord{cal(500, 1500, 2500, -90, 90), cal(500, 1500, 2500, -90, 90)};
}

bool same(const Servo::Calibration& a, const Servo::Calibration& b)
{
    return a.minPulseUs == b.minPulseUs && a.centerPulseUs == b.centerPulseUs &&
           a.maxPulseUs == b.maxPulseUs && a.minDeg == b.minDeg && a.maxDeg == b.maxDeg;
}

struct Rig
{
    FakeServoChannel     panCh;
    FakeServoChannel     tiltCh;
    Servo                pan{panCh, Servo::Calibration{}};
    Servo                tilt{tiltCh, Servo::Calibration{}};
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
    const CalibrationRecord in{cal(550, 1480, 2420, -85, 90), cal(2500, 1500, 500, -45.5f, 60)};
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

// ── Servo::setCalibration ───────────────────────────────────────────────────

TEST(ServoCalibration, NewMappingAppliesToTheNextWrite)
{
    FakeServoChannel ch;
    Servo            s(ch, Servo::Calibration{});
    s.setCalibration(cal(2500, 1500, 500, -90, 90));  // reversed
    ASSERT_TRUE(s.engageAt(45.0f));
    EXPECT_EQ(ch.lastPulseUs, 1000);
}

TEST(ServoCalibration, TargetIsPulledInsideNewLimits)
{
    FakeServoChannel ch;
    Servo            s(ch, Servo::Calibration{});
    ASSERT_TRUE(s.engageAt(0.0f));
    s.setTarget(80.0f);
    s.setCalibration(cal(500, 1500, 2500, -90, 60));
    EXPECT_FLOAT_EQ(s.targetDeg(), 60.0f);
    EXPECT_FLOAT_EQ(s.maxDeg(), 60.0f);
}

// ── Capabilities ────────────────────────────────────────────────────────────

TEST(PanTiltCapabilities, ReportsMotionRanges)
{
    Rig                                     r;
    const ::owlet::core::IPanTilt::Capabilities c = r.head.capabilities();
    EXPECT_FLOAT_EQ(c.speedMin, 1.0f);
    EXPECT_FLOAT_EQ(c.speedMax, 500.0f);
    EXPECT_FLOAT_EQ(c.velocityMax, 500.0f);
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
    r.store.record    = CalibrationRecord{cal(500, 1500, 2500, -85, 90), cal(2500, 1500, 500, -90, 90)};
    r.calib.begin();
    EXPECT_EQ(r.calib.source(), Source::Saved);
    EXPECT_TRUE(same(r.calib.get(Axis::Tilt), r.store.record.tilt));
    EXPECT_FLOAT_EQ(r.head.limits().panMin, -85.0f);
}

TEST(Calibrator, IgnoresASavedRecordThatFailsValidation)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{cal(1500, 1500, 1500, -90, 90), factory().tilt};
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
    EXPECT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -85, 60)), CalResult::Ok);
    EXPECT_FLOAT_EQ(r.head.target().pan, 60.0f);
    EXPECT_FLOAT_EQ(r.head.limits().panMin, -85.0f);
    EXPECT_EQ(r.calib.source(), Source::Modified);
}

TEST(Calibrator, PulsesCannotChangeWhileEnabled)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_EQ(r.calib.set(Axis::Tilt, cal(2500, 1500, 500, -90, 90)), CalResult::Enabled);
    EXPECT_TRUE(same(r.calib.get(Axis::Tilt), factory().tilt));
    EXPECT_EQ(r.calib.source(), Source::Factory);
}

TEST(Calibrator, PulsesChangeWhileDisabledAndDriveTheServo)
{
    Rig r;
    r.calib.begin();
    EXPECT_EQ(r.calib.set(Axis::Tilt, cal(2500, 1500, 500, -90, 90)), CalResult::Ok);
    ASSERT_EQ(r.head.enableAt(0.0f, 45.0f), HeadResult::Ok);
    EXPECT_EQ(r.tiltCh.lastPulseUs, 1000);  // reversed: +45° is below 1500 µs
}

TEST(Calibrator, RejectsInvalidCalibration)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Rig         r;
    r.calib.begin();
    const Servo::Calibration bad[] = {
        cal(300, 1500, 2500, -90, 90),   // pulse below 400
        cal(500, 1500, 2700, -90, 90),   // pulse above 2600
        cal(1400, 1500, 1550, -90, 90),  // span under 200 µs
        cal(500, 2600, 2500, -90, 90),   // centre outside the span
        cal(2500, 400, 500, -90, 90),    // centre outside a reversed span
        cal(500, 1500, 2500, -95, 90),   // limit beyond -90
        cal(500, 1500, 2500, -90, 95),   // limit beyond +90
        cal(500, 1500, 2500, 10, 90),    // min limit above centre
        cal(500, 1500, 2500, -90, -10),  // max limit below centre
        cal(500, 1500, 2500, nan, 90),   // not a number
    };
    for (const Servo::Calibration& c : bad)
    {
        EXPECT_EQ(r.calib.set(Axis::Pan, c), CalResult::Range)
            << c.minPulseUs << "/" << c.centerPulseUs << "/" << c.maxPulseUs << " " << c.minDeg
            << ".." << c.maxDeg;
    }
    EXPECT_TRUE(same(r.calib.get(Axis::Pan), factory().pan));
}

TEST(Calibrator, SaveStoresBothAxes)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Tilt, cal(2500, 1500, 500, -90, 90)), CalResult::Ok);
    EXPECT_EQ(r.calib.save(), CalResult::Ok);
    EXPECT_EQ(r.calib.source(), Source::Saved);
    ASSERT_TRUE(r.store.hasRecord);
    EXPECT_TRUE(same(r.store.record.tilt, cal(2500, 1500, 500, -90, 90)));
    EXPECT_TRUE(same(r.store.record.pan, factory().pan));
}

TEST(Calibrator, SaveFailureIsReported)
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Tilt, cal(2500, 1500, 500, -90, 90)), CalResult::Ok);
    r.store.failSave = true;
    EXPECT_EQ(r.calib.save(), CalResult::Storage);
    EXPECT_EQ(r.calib.source(), Source::Modified);
}

TEST(Calibrator, ResetRestoresFactoryAndErasesTheSavedCopy)
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{factory().pan, cal(2500, 1500, 500, -90, 90)};
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
    r.store.record    = CalibrationRecord{factory().pan, cal(2500, 1500, 500, -90, 90)};
    r.calib.begin();
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_EQ(r.calib.reset(), CalResult::Enabled);
    EXPECT_EQ(r.calib.source(), Source::Saved);
}

TEST(Calibrator, ResetWithFailedEraseChangesNothing)  // review C5
{
    Rig r;
    r.store.hasRecord = true;
    r.store.record    = CalibrationRecord{cal(600, 1500, 2500, -90, 90), factory().tilt};
    r.calib.begin();
    r.store.failErase = true;
    EXPECT_EQ(r.calib.reset(), CalResult::Storage);
    // RAM, flash and the reported source all still agree: the saved record.
    EXPECT_EQ(r.calib.source(), Source::Saved);
    EXPECT_EQ(r.calib.get(Axis::Pan).minPulseUs, 600);
}

// ── Review findings (calibration round, 2026-10-01) ─────────────────────────

TEST(CalibratorReview, InvalidFactoryRecordFallsBackToSafeDefaults)  // C4
{
    FakeServoChannel     a;
    FakeServoChannel     b;
    Servo                pan{a, Servo::Calibration{}};
    Servo                tilt{b, Servo::Calibration{}};
    PanTilt              head{pan, tilt};
    FakeCalibrationStore store;
    Calibrator calib{pan, tilt, head, store,
                     CalibrationRecord{cal(1500, 2600, 1550, 0, 0), factory().tilt}};
    calib.begin();
    EXPECT_FALSE(calib.factoryWasValid());
    EXPECT_TRUE(Calibrator::valid(calib.get(Axis::Pan)));
    EXPECT_TRUE(same(calib.get(Axis::Pan), Servo::Calibration{}));
    EXPECT_EQ(calib.reset(), CalResult::Ok);  // reset re-applies the *safe* factory
    EXPECT_TRUE(Calibrator::valid(calib.get(Axis::Pan)));
}

TEST(CalibratorReview, NarrowingLimitsInVelocityModeNeverJumps)  // C1
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.setWatchdog(0), HeadResult::Ok);
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.setVelocity(-10.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -30, 30)), CalResult::Ok);
    for (int i = 0; i < 300; ++i)
    {
        const float before = r.head.position().pan;
        ASSERT_TRUE(r.head.tick(10).busOk);
        // speed() is the default 90°/s: never more than one step per tick
        EXPECT_LE(before - r.head.position().pan, 90.0f * 0.010f + 1e-3f) << "tick " << i;
    }
    EXPECT_FLOAT_EQ(r.head.position().pan, 30.0f);  // slewed inside, not snapped
}

TEST(CalibratorReview, NarrowingLimitsWhilePinnedSlewsInAndLaterVelocityIsSmooth)  // C1
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.setWatchdog(0), HeadResult::Ok);
    ASSERT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.setVelocity(10.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -30, 30)), CalResult::Ok);
    for (int i = 0; i < 100; ++i)
    {
        ASSERT_TRUE(r.head.tick(10).busOk);
    }
    EXPECT_FLOAT_EQ(r.head.position().pan, 30.0f);
    ASSERT_EQ(r.head.setVelocity(-1.0f, 0.0f), HeadResult::Ok);
    const float before = r.head.position().pan;
    ASSERT_TRUE(r.head.tick(10).busOk);
    EXPECT_NEAR(r.head.position().pan, before - 0.01f, 1e-4f);
}

TEST(CalibratorReview, ReversingWhileDisabledKeepsTheHornWhereItIs)  // C2
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enableAt(45.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.panCh.lastPulseUs, 2000);
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(2500, 1500, 500, -90, 90)), CalResult::Ok);
    EXPECT_FLOAT_EQ(r.head.position().pan, -45.0f);  // same horn, new meaning
    ASSERT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_EQ(r.panCh.lastPulseUs, 2000);            // no snap
}

TEST(CalibratorReview, BareEnableIsRefusedWhenTheHornIsOutsideNewLimits)  // C2
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enableAt(80.0f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -90, 60)), CalResult::Ok);
    const std::size_t writes = r.panCh.writes.size();
    EXPECT_EQ(r.head.enable(), HeadResult::Range);   // would have snapped 20°
    EXPECT_EQ(r.panCh.writes.size(), writes);
    EXPECT_EQ(r.head.state(), ::owlet::core::IPanTilt::State::Disabled);
    EXPECT_EQ(r.head.enableAt(60.0f, 0.0f), HeadResult::Ok);  // explicit pose is the user's call
}

TEST(CalibratorReview, LimitsAreStoredAtTenthOfADegree)  // C7
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -85.06f, 89.94f)), CalResult::Ok);
    EXPECT_FLOAT_EQ(r.calib.get(Axis::Pan).minDeg, -85.1f);
    EXPECT_FLOAT_EQ(r.calib.get(Axis::Pan).maxDeg, 89.9f);
}


TEST(ServoCalibration, PulseToAngleInvertsAngleToPulse)
{
    FakeServoChannel ch;
    for (const Servo::Calibration& c :
         {cal(500, 1500, 2500, -90, 90), cal(600, 1400, 2400, -90, 90), cal(2500, 1500, 500, -90, 90),
          cal(2450, 1520, 580, -90, 90)})
    {
        Servo s(ch, c);
        for (float deg = -90.0f; deg <= 90.0f; deg += 7.5f)
        {
            // Pulses are whole µs, so allow the angle one µs's worth of error.
            const int   spanUs   = std::max(std::abs(c.maxPulseUs - c.centerPulseUs),
                                            std::abs(c.minPulseUs - c.centerPulseUs));
            const float usPerDeg = static_cast<float>(spanUs) / 90.0f;
            EXPECT_NEAR(s.pulseToAngle(s.angleToPulse(deg)), deg, 1.0f / usPerDeg + 1e-3f)
                << c.minPulseUs << "/" << c.centerPulseUs << "/" << c.maxPulseUs << " @ " << deg;
        }
    }
}

TEST(CalibratorReview, HornBeyondTheNewPulseRangeIsTreatedAsUnknown)  // N6
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.head.enableAt(90.0f, 0.0f), HeadResult::Ok);  // horn at 2500 µs
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(700, 1500, 2300, -90, 90)), CalResult::Ok);
    const std::size_t writes = r.panCh.writes.size();
    EXPECT_EQ(r.head.enable(), HeadResult::Range);  // 2500 µs can't be expressed: don't guess
    EXPECT_EQ(r.panCh.writes.size(), writes);
    EXPECT_EQ(r.head.enableAt(90.0f, 0.0f), HeadResult::Ok);  // the caller chooses
    EXPECT_EQ(r.head.disable(), HeadResult::Ok);
    EXPECT_EQ(r.head.enable(), HeadResult::Ok);     // known again after an explicit pose
}

TEST(CalibratorReview, RoundingNoiseAtALimitDoesNotBlockEnable)  // N8
{
    Rig r;
    r.calib.begin();
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -45.5f, 45.5f)), CalResult::Ok);
    ASSERT_EQ(r.head.enableAt(45.5f, 0.0f), HeadResult::Ok);
    ASSERT_EQ(r.head.disable(), HeadResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(510, 1500, 2490, -45.5f, 45.5f)), CalResult::Ok);
    ASSERT_EQ(r.calib.set(Axis::Pan, cal(500, 1500, 2500, -45.5f, 45.5f)), CalResult::Ok);
    EXPECT_EQ(r.head.enable(), HeadResult::Ok);
    EXPECT_NEAR(r.panCh.lastPulseUs, 2006, 1);  // within a µs of where it was
}
