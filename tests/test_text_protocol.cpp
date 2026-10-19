#include "FakeCalibrationStore.hpp"
#include "FakeClock.hpp"
#include "FakeSerialPort.hpp"
#include "ReferenceStepperHead.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/ProtocolMux.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/text/LineWriter.hpp"
#include "owlet/text/TextProtocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

using ::owlet::StepperAxis;
using ::owlet::core::ControlLoop;
using ::owlet::text::TextProtocol;
using ::owlet::core::PanTilt;
using ::owlet::text::LineWriter;
using ::owlet::test::FakeClock;
using ::owlet::test::FakeSerialPort;
using ::owlet::test::ReferenceAxes;

namespace
{

std::string frame(const std::string& body)
{
    LineWriter w;
    w.beginRaw(body);
    return std::string(w.finish());
}

// Expected device line for `body`, without CRLF.
std::string line(const std::string& body)
{
    const std::string f = frame(body);
    return f.substr(0, f.size() - 2);
}

::owlet::core::CalibrationRecord factoryCal(const ReferenceAxes& axes)
{
    return ::owlet::core::CalibrationRecord{axes.pan.defaultCalibration(),
                                            axes.tilt.defaultCalibration()};
}

struct Rig : ReferenceAxes
{
    FakeClock        clock{1000};
    uint32_t&        now = clock.t;
    ::owlet::hal::NullInterruptLock      lock;
    ::owlet::core::StaticEventBus<16, 2> bus{lock, clock};
    ::owlet::core::TroubleLog            trouble;
    PanTilt          head{pan, tilt, &bus};
    FakeSerialPort   port;
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator           calib{pan, tilt, head, store, factoryCal(*this)};
    TextProtocol     proto{port, head, calib, clock, "0.1.0", nullptr, &bus, &trouble};
    ControlLoop      proc{port, head, proto, &bus};

    Rig()
    {
        EXPECT_TRUE(bus.attach(trouble));
        EXPECT_TRUE(bus.attach(proto));
        calib.begin();
        proc.begin(now);
        (void) port.takeLines();
    }

    // Send one request and run the loop once; returns all output lines.
    std::vector<std::string> send(const std::string& body)
    {
        port.rx += frame(body);
        proc.service(now);
        return port.takeLines();
    }

    std::vector<std::string> advance(uint32_t ms, uint32_t step = 10)
    {
        for (uint32_t t = 0; t < ms; t += step)
        {
            now += step;
            proc.service(now);
        }
        return port.takeLines();
    }

    // Expected POS line stamped with the current tick time.
    std::string pos(const std::string& body) const
    {
        return line(body + "," + std::to_string(now));
    }
};

} // namespace

TEST(TextProtocol, AnnouncesReadyOnBegin)
{
    ReferenceAxes    axes;
    PanTilt          head{axes.pan, axes.tilt};
    FakeSerialPort   port;
    FakeClock        clock;
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator           calib{axes.pan, axes.tilt, head, store, factoryCal(axes)};
    TextProtocol     proto{port, head, calib, clock, "0.1.0"};
    proto.begin(0);
    EXPECT_EQ(port.takeLines(), std::vector<std::string>{line("READY,3,0.1.0")});
}

TEST(TextProtocol, PingReturnsVersions)
{
    Rig r;
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,3,0.1.0")});
}

TEST(TextProtocol, PosReportsDisabledAtBoot)
{
    Rig r;
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,0.0,0.0,0.0,0.0,D")});
}

TEST(TextProtocol, MoveWhileDisabledIsNakDis)
{
    Rig r;
    EXPECT_EQ(r.send("MOVE,10,10"), std::vector<std::string>{line("NAK,MOVE,DIS")});
    (void) r.advance(100);
    EXPECT_FALSE(r.anyStepped());
}

TEST(TextProtocol, EnableMoveArriveSequence)
{
    Rig r;
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.send("SPEED,100"), std::vector<std::string>{line("ACK,SPEED")});
    EXPECT_EQ(r.send("MOVE,30,-15"), std::vector<std::string>{line("ACK,MOVE")});

    const std::vector<std::string> mid = r.advance(100);
    EXPECT_TRUE(mid.empty());
    const std::vector<std::string> moving = r.send("POS");
    ASSERT_EQ(moving.size(), 1u);
    EXPECT_NE(moving[0].find(",30.0,-15.0,M,"), std::string::npos) << moving[0];
    EXPECT_GT(r.head.position().pan, 0.0f);
    EXPECT_LT(r.head.position().pan, 30.0f);

    // Ramping up and down takes longer than distance / speed.
    const std::vector<std::string> done = r.advance(1000);
    EXPECT_EQ(done, std::vector<std::string>{line("ARRIVED,30.0,-15.0")});
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,30.0,-15.0,30.0,-15.0,H")});
}

TEST(TextProtocol, ArrivedFiresEvenForZeroLengthMove)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("HOME"), std::vector<std::string>{line("ACK,HOME")});
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ARRIVED,0.0,0.0")});
    EXPECT_TRUE(r.advance(100).empty());
}

TEST(TextProtocol, StopCancelsPendingArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,90,0");
    (void) r.advance(50);
    EXPECT_EQ(r.send("STOP"), std::vector<std::string>{line("ACK,STOP")});
    EXPECT_TRUE(r.advance(2000).empty());
}

TEST(TextProtocol, RangeAndFormatErrors)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("MOVE,91,0"), std::vector<std::string>{line("NAK,MOVE,RNG")});
    EXPECT_EQ(r.send("MOVE,1"), std::vector<std::string>{line("NAK,MOVE,FMT")});
    EXPECT_EQ(r.send("SPEED,0"), std::vector<std::string>{line("NAK,SPEED,RNG")});
    EXPECT_EQ(r.send("STREAM,51"), std::vector<std::string>{line("NAK,STREAM,RNG")});
    EXPECT_EQ(r.send("FLY"), std::vector<std::string>{line("ERR,UNK")});
}

TEST(TextProtocol, FrameLevelErrors)
{
    Rig r;
    r.port.rx += "$PING*11\r\n";
    r.port.rx += "$PING*1\r\n";
    r.port.rx += "$" + std::string(120, 'A') + "\r\n";
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("ERR,CHK"), line("ERR,SYN"), line("ERR,LEN")}));
}

TEST(TextProtocol, IgnoresBootNoiseBeforeFirstFrame)
{
    Rig r;
    r.port.rx += "boot rom v2\r\nbuild: Mar 27 2021\r\n";
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,3,0.1.0")});
}

TEST(TextProtocol, SeveralRequestsInOneReadAnsweredInOrder)
{
    Rig r;
    r.port.rx += frame("PING") + frame("LIMITS") + frame("SPEED");
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("PONG,3,0.1.0"),
                                        line("LIMITS,-90.0,90.0,-60.0,60.0"),
                                        line("SPEED,90.0")}));
}

TEST(TextProtocol, StreamEmitsPosAtRequestedRate)
{
    Rig r;
    EXPECT_EQ(r.send("STREAM,10"), std::vector<std::string>{line("ACK,STREAM")});
    const std::vector<std::string> lines = r.advance(1000);
    EXPECT_EQ(lines.size(), 10u);
    uint32_t prev = 0;
    for (const std::string& l : lines)
    {
        ASSERT_EQ(l.rfind("$POS,0.0,0.0,0.0,0.0,D,", 0), 0u) << l;
        const uint32_t t = static_cast<uint32_t>(std::stoul(l.substr(23, l.find('*') - 23)));
        if (prev != 0)
        {
            EXPECT_EQ(t - prev, 100u);
        }
        prev = t;
    }
    EXPECT_EQ(r.send("STREAM,0"), std::vector<std::string>{line("ACK,STREAM")});
    EXPECT_TRUE(r.advance(1000).empty());
}

TEST(TextProtocol, DisableGoesLimpAndReportsD)
{
    Rig r;
    (void) r.send("ENABLE,20,10");
    EXPECT_EQ(r.send("DISABLE"), std::vector<std::string>{line("ACK,DISABLE")});
    EXPECT_FALSE(r.anyPowered());
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,20.0,10.0,20.0,10.0,D")});
}

TEST(TextProtocol, StalledStepGeneratorReportsErrBusOnceUntilRecovered)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,10,0");
    r.panSteps.busy = true;
    EXPECT_EQ(r.advance(StepperAxis::kStallReportMs), std::vector<std::string>{line("ERR,BUS")});
    r.panSteps.busy = false;
    const std::vector<std::string> rest = r.advance(1000);
    EXPECT_EQ(rest, (std::vector<std::string>{line("EVT,BUSOK,-,0"), line("ARRIVED,10.0,0.0")}))
        << "the recovery is announced once, then the move finishes";
}

TEST(TextProtocol, LargeTimeGapIsCappedSoHeadDoesNotJump)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("SPEED,100");
    (void) r.send("MOVE,90,0");
    r.now += 5000;  // e.g. the loop was starved
    r.proc.service(r.now);
    (void) r.port.takeLines();
    EXPECT_LE(r.head.position().pan, 100.0f * PanTilt::kMaxStepMs / 1000.0f + 1e-3f);
}

TEST(TextProtocol, ClaimsDollarAnywhereAndNothingElse)
{
    Rig r;
    EXPECT_TRUE(r.proto.claims('$', true));
    EXPECT_TRUE(r.proto.claims('$', false));
    EXPECT_FALSE(r.proto.claims('G', true));
    EXPECT_FALSE(r.proto.claims('\n', true));
}

TEST(TextProtocol, ReleasesStreamAtEndOfLine)
{
    Rig r;
    const std::string f = frame("PING");
    const auto*       d = reinterpret_cast<const uint8_t*>(f.data());
    const ::owlet::core::ConsumeResult partial = r.proto.consume(d, 4);
    EXPECT_EQ(partial.consumed, 4u);
    EXPECT_FALSE(partial.released);
    const ::owlet::core::ConsumeResult rest = r.proto.consume(d + 4, f.size() - 4);
    EXPECT_TRUE(rest.released);
    EXPECT_EQ(rest.consumed, f.size() - 4 - 1);  // releases on '\r', leaves '\n'
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("PONG,3,0.1.0")});
}

// ── Protocol v2: velocity, watchdog, clock ──────────────────────────────────

TEST(TextProtocol, VelocityRequiresEnable)
{
    Rig r;
    EXPECT_EQ(r.send("VEL,10,0"), std::vector<std::string>{line("NAK,VEL,DIS")});
}

TEST(TextProtocol, VelocityMovesWithoutArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("VEL,20,-10"), std::vector<std::string>{line("ACK,VEL")});
    EXPECT_TRUE(r.advance(100).empty());
    // 0.1 s at the rate, less the ramp up to it.
    EXPECT_NEAR(r.head.position().pan, 2.0f, 0.2f);
    EXPECT_NEAR(r.head.position().tilt, -1.0f, 0.1f);
    const std::vector<std::string> moving = r.send("POS");
    ASSERT_EQ(moving.size(), 1u);
    EXPECT_NE(moving[0].find(",M," + std::to_string(r.now)), std::string::npos) << moving[0];
    EXPECT_EQ(r.send("VEL,0,0"), std::vector<std::string>{line("ACK,VEL")});
    EXPECT_TRUE(r.advance(2000).empty());  // no ARRIVED, no TIMEOUT
}

TEST(TextProtocol, VelocityErrors)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("VEL,241,0"), std::vector<std::string>{line("NAK,VEL,RNG")});
    EXPECT_EQ(r.send("VEL,1"), std::vector<std::string>{line("NAK,VEL,FMT")});
}

TEST(TextProtocol, WatchdogTimeoutIsReportedOnce)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("VEL,10,0");
    EXPECT_EQ(r.advance(600), std::vector<std::string>{line("TIMEOUT,VEL")});
    // Braking gives back what the ramp up cost: about 0.5 s at 10°/s.
    EXPECT_NEAR(r.head.position().pan, 5.0f, 0.1f);
    const std::vector<std::string> held = r.send("POS");
    ASSERT_EQ(held.size(), 1u);
    EXPECT_NE(held[0].find(",H," + std::to_string(r.now)), std::string::npos) << held[0];
    EXPECT_TRUE(r.advance(1000).empty());
}

TEST(TextProtocol, WatchdogQueryAndSet)
{
    Rig r;
    EXPECT_EQ(r.send("WATCHDOG"), std::vector<std::string>{line("WATCHDOG,500")});
    EXPECT_EQ(r.send("WATCHDOG,1000"), std::vector<std::string>{line("ACK,WATCHDOG")});
    EXPECT_EQ(r.send("WATCHDOG"), std::vector<std::string>{line("WATCHDOG,1000")});
    EXPECT_EQ(r.send("WATCHDOG,10"), std::vector<std::string>{line("NAK,WATCHDOG,RNG")});
    EXPECT_EQ(r.send("WATCHDOG,1.5"), std::vector<std::string>{line("NAK,WATCHDOG,FMT")});
    EXPECT_EQ(r.send("WATCHDOG,0"), std::vector<std::string>{line("ACK,WATCHDOG")});
}

TEST(TextProtocol, VelocityCancelsPendingArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,90,0");
    (void) r.send("VEL,0,0");
    EXPECT_TRUE(r.advance(3000).empty());
}

TEST(TextProtocol, ClockReportsCurrentTimeNotTickTime)
{
    Rig r;
    r.proc.service(r.now);
    (void) r.port.takeLines();
    r.clock.t += 7;  // time passes between loop ticks
    const std::string f = frame("CLOCK");
    (void) r.proto.consume(reinterpret_cast<const uint8_t*>(f.data()), f.size());
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("CLOCK,1007")});
}

TEST(TextProtocol, PosIsStampedWithTheTickThatProducedIt)
{
    Rig r;
    r.proc.service(r.now);  // pose computed at t=1000
    r.clock.t += 7;
    const std::string f = frame("POS");
    (void) r.proto.consume(reinterpret_cast<const uint8_t*>(f.data()), f.size());
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("POS,0.0,0.0,0.0,0.0,D,1000")});
}

// ── Review findings (adversarial review 2026-10-01) ─────────────────────────

TEST(TextProtocolReview, BareEnableDuringVelocityDoesNotArmArrival)  // M1
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("VEL,10,0");
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.advance(1000), std::vector<std::string>{line("TIMEOUT,VEL")});  // no ARRIVED
}

TEST(TextProtocolReview, EnableAtPoseWhileEnabledAlwaysArrives)  // M1
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("ENABLE,0,0"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ARRIVED,0.0,0.0")});
}

TEST(TextProtocolReview, EnableFromDisabledNeverArrives)  // M1
{
    Rig r;
    (void) r.send("ENABLE,10,10");
    EXPECT_TRUE(r.advance(500).empty());
}

TEST(TextProtocolReview, StrayDollarBeforeBinaryIsHandedBackSilently)  // M2
{
    Rig r;
    const std::string in = std::string("$") + '\xFF';
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(res.consumed, 1u);  // 0xFF left for another protocol
    EXPECT_TRUE(r.port.takeLines().empty());
}

TEST(TextProtocolReview, GarbledRequestStillGetsAReply)  // N1
{
    for (const std::string& bad : {std::string("$MOVE,1\xC2\xB0,2*00\r\n"),  // UTF-8 degree sign
                                   std::string("$PING\x7F*10\r\n"),
                                   std::string("$PING\x01\r\n")})
    {
        Rig r;
        r.port.rx += bad;
        r.proc.service(r.now);
        EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("ERR,SYN")}) << bad;
        // ...and the next request is unaffected.
        EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,3,0.1.0")});
    }
}

TEST(TextProtocolReview, NonTextInsideAFrameHandsTheByteBackAfterReporting)  // N1 + M2
{
    Rig r;
    const std::string in = std::string("$PI") + '\xFF' + "NG";
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(res.consumed, 3u);
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("ERR,SYN")});
}

TEST(TextProtocolReview, OverflowHandsTheStreamBack)  // M2
{
    Rig r;
    const std::string in = "$" + std::string(100, 'A');
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("ERR,LEN")});
}

// ── Protocol v3: capabilities and calibration ───────────────────────────────

TEST(TextCalibration, CapsReportsMotionRanges)
{
    Rig r;
    EXPECT_EQ(r.send("CAPS"), std::vector<std::string>{line("CAPS,1.0,240.0,240.0,50,5000,50")});
}

TEST(TextCalibration, CalQueryReportsAxisCalibration)
{
    Rig r;
    EXPECT_EQ(r.send("CAL,PAN"), std::vector<std::string>{line("CAL,PAN,0,0,0,-90.0,90.0,N,F")});
    EXPECT_EQ(r.send("CAL,TILT"), std::vector<std::string>{line("CAL,TILT,0,0,0,-60.0,60.0,N,F")});
}

TEST(TextCalibration, SetLimitsSaveAndQuery)
{
    Rig r;
    EXPECT_EQ(r.send("CAL,TILT,0,0,0,-45,120"), std::vector<std::string>{line("ACK,CAL")});
    EXPECT_EQ(r.send("CAL,TILT"),
              std::vector<std::string>{line("CAL,TILT,0,0,0,-45.0,120.0,N,M")});
    EXPECT_EQ(r.send("CAL,SAVE"), std::vector<std::string>{line("ACK,CAL")});
    EXPECT_TRUE(r.store.hasRecord);
    EXPECT_EQ(r.send("CAL,TILT"),
              std::vector<std::string>{line("CAL,TILT,0,0,0,-45.0,120.0,N,S")});
}

TEST(TextCalibration, LimitsChangeWhileEnabledAndShowInLimits)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("CAL,PAN,0,0,0,-85,90"), std::vector<std::string>{line("ACK,CAL")});
    EXPECT_EQ(r.send("CAL,TILT,0,0,0,-45,45"), std::vector<std::string>{line("ACK,CAL")});
    EXPECT_EQ(r.send("LIMITS"), std::vector<std::string>{line("LIMITS,-85.0,90.0,-45.0,45.0")});
}

TEST(TextCalibration, Errors)
{
    Rig r;
    // A stepper has no drive numbers to tune: anything but zeros is refused.
    EXPECT_EQ(r.send("CAL,PAN,500,1500,2500,-90,90"), std::vector<std::string>{line("NAK,CAL,RNG")});
    EXPECT_EQ(r.send("CAL,PAN,0,1,0,-90,90"), std::vector<std::string>{line("NAK,CAL,RNG")});
    // 65536 would wrap to 0 in a uint16_t and pass validation if cast blindly.
    EXPECT_EQ(r.send("CAL,PAN,65536,0,0,-90,90"), std::vector<std::string>{line("NAK,CAL,RNG")});
    EXPECT_EQ(r.send("CAL,PAN,0,0,0,-181,90"), std::vector<std::string>{line("NAK,CAL,RNG")})
        << "beyond what the mechanism allows";
    EXPECT_EQ(r.send("CAL,TILT,0,0,0,-45,136"), std::vector<std::string>{line("NAK,CAL,RNG")});
    EXPECT_EQ(r.send("CAL,PAN,0"), std::vector<std::string>{line("NAK,CAL,FMT")});
    EXPECT_EQ(r.send("CAL,FOO"), std::vector<std::string>{line("NAK,CAL,FMT")});
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("CAL,RESET"), std::vector<std::string>{line("NAK,CAL,ENA")});
    r.store.failSave = true;
    EXPECT_EQ(r.send("CAL,SAVE"), std::vector<std::string>{line("NAK,CAL,STO")});
}

TEST(TextCalibration, ResetRestoresFactory)
{
    Rig r;
    (void) r.send("CAL,TILT,0,0,0,-45,45");
    (void) r.send("CAL,SAVE");
    EXPECT_EQ(r.send("CAL,RESET"), std::vector<std::string>{line("ACK,CAL")});
    EXPECT_EQ(r.send("CAL,TILT"), std::vector<std::string>{line("CAL,TILT,0,0,0,-60.0,60.0,N,F")});
    EXPECT_FALSE(r.store.hasRecord);
}

// ── Driver diagnostics ($DIAG) ──────────────────────────────────────────────

#include "owlet/core/IDiagnostics.hpp"

namespace
{

class FakeDiagnostics final : public ::owlet::core::IDiagnostics
{
public:
    [[nodiscard]] ::owlet::hal::AxisDiagnostics read(::owlet::core::Axis axis) const override
    {
        return axis == ::owlet::core::Axis::Pan ? pan : tilt;
    }

    ::owlet::hal::AxisDiagnostics pan{};
    ::owlet::hal::AxisDiagnostics tilt{};
};

} // namespace

TEST(TextDiagnostics, AHeadWithNoDriverDataSaysSo)
{
    Rig r;  // built without a diagnostics source, like a head on standalone drivers
    EXPECT_EQ(r.send("DIAG,PAN"), std::vector<std::string>{line("DIAG,PAN,N,0,0,0,0")});
    EXPECT_EQ(r.send("DIAG,TILT"), std::vector<std::string>{line("DIAG,TILT,N,0,0,0,0")});
}

TEST(TextDiagnostics, ReportsWhatTheDriverSays)
{
    ReferenceAxes                       axes;
    PanTilt                             head{axes.pan, axes.tilt};
    FakeSerialPort                      port;
    FakeClock                           clock{1000};
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator           calib{axes.pan, axes.tilt, head, store, factoryCal(axes)};
    FakeDiagnostics                     diag;
    TextProtocol                        proto{port, head, calib, clock, "0.1.0", &diag};
    ControlLoop                         loop{port, head, proto};
    loop.begin(clock.t);
    (void) port.takeLines();

    diag.pan.available = true;
    diag.pan.faults    = ::owlet::hal::AxisDiagnostics::kOverTemp | ::owlet::hal::AxisDiagnostics::kShortA;
    diag.pan.stalled   = true;
    diag.pan.load      = 37;
    diag.pan.currentMa = 459;
    port.rx += frame("DIAG,PAN");
    loop.service(clock.t);
    EXPECT_EQ(port.takeLines(), std::vector<std::string>{line("DIAG,PAN,U,6,1,37,459")});

    port.rx += frame("DIAG,TILT");  // tilt's driver is not answering
    loop.service(clock.t);
    EXPECT_EQ(port.takeLines(), std::vector<std::string>{line("DIAG,TILT,N,0,0,0,0")});
}

TEST(TextDiagnostics, NeedsExactlyOneAxis)
{
    Rig r;
    EXPECT_EQ(r.send("DIAG"), std::vector<std::string>{line("NAK,DIAG,FMT")});
    EXPECT_EQ(r.send("DIAG,ROLL"), std::vector<std::string>{line("NAK,DIAG,FMT")});
    EXPECT_EQ(r.send("DIAG,PAN,1"), std::vector<std::string>{line("NAK,DIAG,FMT")});
}

// ── Events on the wire ──────────────────────────────────────────────────────

namespace
{

using ::owlet::core::Event;
using ::owlet::core::EventId;

// Posts one event and returns what the protocol wrote for it.
std::vector<std::string> announce(Rig& r, EventId id, uint8_t axis, uint16_t arg)
{
    (void) r.bus.post(id, axis, arg);
    return r.advance(10);
}

} // namespace

TEST(TextEvents, EachKindOfEventHasItsLine)
{
    Rig r;
    using V = std::vector<std::string>;
    EXPECT_EQ(announce(r, EventId::LimitHit, 0, 1), V{line("EVT,LIMIT,PAN,MAX")});
    EXPECT_EQ(announce(r, EventId::LimitHit, 1, 0), V{line("EVT,LIMIT,TILT,MIN")});
    EXPECT_EQ(announce(r, EventId::Stalled, 1, 0), V{line("EVT,STALL,TILT,0")});
    EXPECT_EQ(announce(r, EventId::DriverFaultChanged, 0, 6), V{line("EVT,FAULT,PAN,6")});
    EXPECT_EQ(announce(r, EventId::DriverOnline, 0, 0), V{line("EVT,ONLINE,PAN,0")});
    EXPECT_EQ(announce(r, EventId::DriverOffline, 1, 0), V{line("EVT,OFFLINE,TILT,0")});
    EXPECT_EQ(announce(r, EventId::PositionLost, 0, 0), V{line("EVT,LOST,PAN,0")});
    EXPECT_EQ(announce(r, EventId::Homed, Event::kHead, 0), V{line("EVT,HOMED,-,0")});
    EXPECT_EQ(announce(r, EventId::HomingFailed, 1, 2), V{line("EVT,HOMEFAIL,TILT,2")});
    EXPECT_EQ(announce(r, EventId::BusOk, Event::kHead, 0), V{line("EVT,BUSOK,-,0")});
}

TEST(TextEvents, TheOlderLinesAreNotRepeatedAsEvt)
{
    Rig r;
    using V = std::vector<std::string>;
    EXPECT_EQ(announce(r, EventId::WatchdogExpired, Event::kHead, 0), V{line("TIMEOUT,VEL")});
    EXPECT_EQ(announce(r, EventId::BusFault, Event::kHead, 0), V{line("ERR,BUS")});
    EXPECT_TRUE(announce(r, EventId::Arrived, Event::kHead, 0).empty()) << "nobody on this link asked for a move";
}

TEST(TextEvents, CanBeTurnedOffAndOnAgain)
{
    Rig r;
    using V = std::vector<std::string>;
    EXPECT_EQ(r.send("EVENTS,0"), V{line("ACK,EVENTS")});
    EXPECT_TRUE(announce(r, EventId::Stalled, 0, 0).empty());
    EXPECT_EQ(announce(r, EventId::WatchdogExpired, Event::kHead, 0), V{line("TIMEOUT,VEL")})
        << "the lines clients already rely on are not affected";
    EXPECT_EQ(r.send("EVENTS,1"), V{line("ACK,EVENTS")});
    EXPECT_EQ(announce(r, EventId::Stalled, 0, 0), V{line("EVT,STALL,PAN,0")});
    EXPECT_EQ(r.send("EVENTS,2"), V{line("NAK,EVENTS,RNG")});
    EXPECT_EQ(r.send("EVENTS"), V{line("NAK,EVENTS,FMT")});
}

TEST(TextEvents, ALostEventIsReportedWithTheCount)
{
    Rig r;
    for (int i = 0; i < 20; ++i)  // more than the queue of 16 holds
    {
        (void) r.bus.post(EventId::DriverOnline, 0, 0);
    }
    const std::vector<std::string> lines = r.advance(10);
    ASSERT_EQ(lines.size(), 17u);
    EXPECT_EQ(lines.back(), line("EVT,OVERFLOW,-,4"));
}

TEST(TextEvents, StatReportsTheQueueCounters)
{
    Rig r;
    EXPECT_EQ(r.send("STAT"), std::vector<std::string>{line("STAT,0,0,0,0,0")});
    (void) announce(r, EventId::Stalled, 0, 0);
    (void) r.bus.post(EventId::DriverOnline, 0, 0);
    (void) r.bus.post(EventId::DriverOnline, 1, 0);
    (void) r.advance(10);
    EXPECT_EQ(r.send("STAT"), std::vector<std::string>{line("STAT,3,0,2,0,0")});
}

// ── Trouble codes ───────────────────────────────────────────────────────────

TEST(TextTrouble, NothingRecordedAtPowerUp)
{
    Rig r;
    EXPECT_EQ(r.send("DTC"), std::vector<std::string>{line("DTC,0")});
    EXPECT_EQ(r.send("DTC,0"), std::vector<std::string>{line("NAK,DTC,RNG")});
}

TEST(TextTrouble, ListsWhatWentWrongWithCountsAndTimes)
{
    Rig r;  // the rig's clock starts at 1000 ms
    (void) announce(r, EventId::DriverFaultChanged, 1, 6);
    (void) r.advance(500);
    (void) announce(r, EventId::DriverFaultChanged, 1, 2);
    (void) announce(r, EventId::LimitHit, 0, 1);
    EXPECT_EQ(r.send("DTC"), std::vector<std::string>{line("DTC,2")});
    EXPECT_EQ(r.send("DTC,0"), std::vector<std::string>{line("DTC,0,21,TILT,2,2,1000,1510")});
    EXPECT_EQ(r.send("DTC,1"), std::vector<std::string>{line("DTC,1,30,PAN,1,1,1520,1520")});
    EXPECT_EQ(r.send("DTC,2"), std::vector<std::string>{line("NAK,DTC,RNG")});
}

TEST(TextTrouble, RecordsTheHeadsOwnTroubleToo)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("VEL,10,0");
    (void) r.advance(600);  // no further command: the watchdog stops it
    EXPECT_EQ(r.send("DTC"), std::vector<std::string>{line("DTC,1")});
    const std::vector<std::string> record = r.send("DTC,0");
    ASSERT_EQ(record.size(), 1u);
    EXPECT_EQ(record[0].rfind("$DTC,0,11,-,1,0,", 0), 0u) << record[0];
}

TEST(TextTrouble, ClearEmptiesTheList)
{
    Rig r;
    (void) announce(r, EventId::Stalled, 0, 0);
    EXPECT_EQ(r.send("DTC,CLEAR"), std::vector<std::string>{line("ACK,DTC")});
    EXPECT_EQ(r.send("DTC"), std::vector<std::string>{line("DTC,0")});
}

TEST(TextTrouble, MalformedRequestsAreRefused)
{
    Rig r;
    EXPECT_EQ(r.send("DTC,WIPE"), std::vector<std::string>{line("NAK,DTC,FMT")});
    EXPECT_EQ(r.send("DTC,0,1"), std::vector<std::string>{line("NAK,DTC,FMT")});
}

// ── Homing and a lost position on the wire ──────────────────────────────────

TEST(TextHoming, AHeadWithoutSwitchesSaysItCannotSeek)
{
    Rig r;
    EXPECT_EQ(r.send("SEEK"), std::vector<std::string>{line("NAK,SEEK,UNS")});
    EXPECT_EQ(r.send("SEEK,1"), std::vector<std::string>{line("NAK,SEEK,FMT")});
}

TEST(TextCalibrationSave, IsRefusedWhileTheHeadIsMoving)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,40,0");
    (void) r.advance(20);
    EXPECT_EQ(r.send("CAL,SAVE"), std::vector<std::string>{line("NAK,CAL,MOV")});
    (void) r.advance(3000);
    EXPECT_EQ(r.send("CAL,SAVE"), std::vector<std::string>{line("ACK,CAL")});
}

// ── The whole stack, down to the step pulses ────────────────────────────────
// ControlLoop → ProtocolMux → TextProtocol → PanTilt → StepperAxis → the step
// generator and driver enable the Pico would be driving.

TEST(TextStack, CommandsArriveAsStepPulsesAndDriverPower)
{
    ReferenceAxes                       axes;
    PanTilt                             head{axes.pan, axes.tilt};
    FakeSerialPort                      port;
    FakeClock                           clock;
    ::owlet::test::FakeCalibrationStore store;
    ::owlet::core::Calibrator           calib{axes.pan, axes.tilt, head, store, factoryCal(axes)};
    TextProtocol                        native{port, head, calib, clock, "test"};
    ::owlet::core::ProtocolMux<1>       mux{{&native}};
    ControlLoop                         loop{port, head, mux};
    calib.begin();
    loop.begin(clock.t);

    const auto send = [&](const std::string& body)
    {
        port.rx += frame(body);
        loop.service(clock.t);
    };
    const auto run = [&](uint32_t ms)
    {
        for (uint32_t t = 0; t < ms; t += 2)
        {
            clock.t += 2;
            loop.service(clock.t);
        }
    };

    send("MOVE,90,-60");
    run(500);
    EXPECT_FALSE(axes.anyPowered());
    EXPECT_FALSE(axes.anyStepped());

    send("ENABLE,20,10");
    EXPECT_TRUE(axes.panEnable.on);
    EXPECT_TRUE(axes.tiltEnable.on);
    EXPECT_FALSE(axes.anyStepped()) << "ENABLE names the pose; it never moves the shaft";

    send("SPEED,240");
    send("MOVE,90,-60");
    run(2000);
    const auto steps = [](float deg)
    { return std::lround(deg * ::owlet::test::referencePan().stepsPerDeg); };
    EXPECT_EQ(axes.panSteps.total, steps(90.0f) - steps(20.0f));
    EXPECT_EQ(axes.tiltSteps.total, steps(-60.0f) - steps(10.0f));

    send("DISABLE");
    EXPECT_FALSE(axes.anyPowered());
}
