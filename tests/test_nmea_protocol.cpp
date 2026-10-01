#include "FakeClock.hpp"
#include "FakeSerialPort.hpp"
#include "FakeServoChannel.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/nmea/LineWriter.hpp"
#include "owlet/nmea/NmeaProtocol.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using ::owlet::Servo;
using ::owlet::core::ControlLoop;
using ::owlet::nmea::NmeaProtocol;
using ::owlet::core::PanTilt;
using ::owlet::nmea::LineWriter;
using ::owlet::test::FakeClock;
using ::owlet::test::FakeSerialPort;
using ::owlet::test::FakeServoChannel;

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

struct Rig
{
    FakeServoChannel panCh;
    FakeServoChannel tiltCh;
    Servo            pan{panCh, Servo::Calibration{}};
    Servo            tilt{tiltCh, Servo::Calibration{}};
    PanTilt          head{pan, tilt};
    FakeSerialPort   port;
    FakeClock        clock{1000};
    uint32_t&        now = clock.t;
    NmeaProtocol     proto{port, head, clock, "0.1.0"};
    ControlLoop      proc{port, head, proto};

    Rig()
    {
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

TEST(NmeaProtocol, AnnouncesReadyOnBegin)
{
    FakeServoChannel a;
    FakeServoChannel b;
    Servo            pan{a, Servo::Calibration{}};
    Servo            tilt{b, Servo::Calibration{}};
    PanTilt          head{pan, tilt};
    FakeSerialPort   port;
    FakeClock        clock;
    NmeaProtocol     proto{port, head, clock, "0.1.0"};
    proto.begin(0);
    EXPECT_EQ(port.takeLines(), std::vector<std::string>{line("READY,2,0.1.0")});
}

TEST(NmeaProtocol, PingReturnsVersions)
{
    Rig r;
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,2,0.1.0")});
}

TEST(NmeaProtocol, PosReportsDisabledAtBoot)
{
    Rig r;
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,0.0,0.0,0.0,0.0,D")});
}

TEST(NmeaProtocol, MoveWhileDisabledIsNakDis)
{
    Rig r;
    EXPECT_EQ(r.send("MOVE,10,10"), std::vector<std::string>{line("NAK,MOVE,DIS")});
    EXPECT_TRUE(r.panCh.writes.empty());
}

TEST(NmeaProtocol, EnableMoveArriveSequence)
{
    Rig r;
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.send("SPEED,100"), std::vector<std::string>{line("ACK,SPEED")});
    EXPECT_EQ(r.send("MOVE,30,-15"), std::vector<std::string>{line("ACK,MOVE")});

    const std::vector<std::string> mid = r.advance(100);
    EXPECT_TRUE(mid.empty());
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,10.0,-10.0,30.0,-15.0,M")});

    const std::vector<std::string> done = r.advance(300);
    EXPECT_EQ(done, std::vector<std::string>{line("ARRIVED,30.0,-15.0")});
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,30.0,-15.0,30.0,-15.0,H")});
}

TEST(NmeaProtocol, ArrivedFiresEvenForZeroLengthMove)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("HOME"), std::vector<std::string>{line("ACK,HOME")});
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ARRIVED,0.0,0.0")});
    EXPECT_TRUE(r.advance(100).empty());
}

TEST(NmeaProtocol, StopCancelsPendingArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,90,0");
    (void) r.advance(50);
    EXPECT_EQ(r.send("STOP"), std::vector<std::string>{line("ACK,STOP")});
    EXPECT_TRUE(r.advance(2000).empty());
}

TEST(NmeaProtocol, RangeAndFormatErrors)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("MOVE,91,0"), std::vector<std::string>{line("NAK,MOVE,RNG")});
    EXPECT_EQ(r.send("MOVE,1"), std::vector<std::string>{line("NAK,MOVE,FMT")});
    EXPECT_EQ(r.send("SPEED,0"), std::vector<std::string>{line("NAK,SPEED,RNG")});
    EXPECT_EQ(r.send("STREAM,51"), std::vector<std::string>{line("NAK,STREAM,RNG")});
    EXPECT_EQ(r.send("FLY"), std::vector<std::string>{line("ERR,UNK")});
}

TEST(NmeaProtocol, FrameLevelErrors)
{
    Rig r;
    r.port.rx += "$PING*11\r\n";
    r.port.rx += "$PING*1\r\n";
    r.port.rx += "$" + std::string(120, 'A') + "\r\n";
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("ERR,CHK"), line("ERR,SYN"), line("ERR,LEN")}));
}

TEST(NmeaProtocol, IgnoresBootNoiseBeforeFirstFrame)
{
    Rig r;
    r.port.rx += "ESP-ROM:esp32s3\r\nbuild: Mar 27 2021\r\n";
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,2,0.1.0")});
}

TEST(NmeaProtocol, SeveralRequestsInOneReadAnsweredInOrder)
{
    Rig r;
    r.port.rx += frame("PING") + frame("LIMITS") + frame("SPEED");
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("PONG,2,0.1.0"),
                                        line("LIMITS,-90.0,90.0,-90.0,90.0"),
                                        line("SPEED,90.0")}));
}

TEST(NmeaProtocol, StreamEmitsPosAtRequestedRate)
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

TEST(NmeaProtocol, DisableGoesLimpAndReportsD)
{
    Rig r;
    (void) r.send("ENABLE,20,10");
    EXPECT_EQ(r.send("DISABLE"), std::vector<std::string>{line("ACK,DISABLE")});
    EXPECT_TRUE(r.panCh.idle);
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,20.0,10.0,20.0,10.0,D")});
}

TEST(NmeaProtocol, EnableBusFailureIsNakBus)
{
    Rig r;
    r.panCh.failNext = true;
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("NAK,ENABLE,BUS")});
}

TEST(NmeaProtocol, SlewBusFailureReportsErrBusOnceUntilRecovered)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,10,0");
    r.panCh.failNext = true;
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ERR,BUS")});
    const std::vector<std::string> rest = r.advance(1000);
    EXPECT_EQ(rest, std::vector<std::string>{line("ARRIVED,10.0,0.0")});
}

TEST(NmeaProtocol, LargeTimeGapIsCappedSoHeadDoesNotJump)
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

TEST(NmeaProtocol, ClaimsDollarAnywhereAndNothingElse)
{
    Rig r;
    EXPECT_TRUE(r.proto.claims('$', true));
    EXPECT_TRUE(r.proto.claims('$', false));
    EXPECT_FALSE(r.proto.claims('G', true));
    EXPECT_FALSE(r.proto.claims('\n', true));
}

TEST(NmeaProtocol, ReleasesStreamAtEndOfLine)
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
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("PONG,2,0.1.0")});
}

// ── Protocol v2: velocity, watchdog, clock ──────────────────────────────────

TEST(NmeaProtocol, VelocityRequiresEnable)
{
    Rig r;
    EXPECT_EQ(r.send("VEL,10,0"), std::vector<std::string>{line("NAK,VEL,DIS")});
}

TEST(NmeaProtocol, VelocityMovesWithoutArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("VEL,20,-10"), std::vector<std::string>{line("ACK,VEL")});
    EXPECT_TRUE(r.advance(100).empty());
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,2.0,-1.0,2.0,-1.0,M")});
    EXPECT_EQ(r.send("VEL,0,0"), std::vector<std::string>{line("ACK,VEL")});
    EXPECT_TRUE(r.advance(2000).empty());  // no ARRIVED, no TIMEOUT
}

TEST(NmeaProtocol, VelocityErrors)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("VEL,600,0"), std::vector<std::string>{line("NAK,VEL,RNG")});
    EXPECT_EQ(r.send("VEL,1"), std::vector<std::string>{line("NAK,VEL,FMT")});
}

TEST(NmeaProtocol, WatchdogTimeoutIsReportedOnce)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("VEL,10,0");
    EXPECT_EQ(r.advance(600), std::vector<std::string>{line("TIMEOUT,VEL")});
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{r.pos("POS,5.0,0.0,5.0,0.0,H")});
    EXPECT_TRUE(r.advance(1000).empty());
}

TEST(NmeaProtocol, WatchdogQueryAndSet)
{
    Rig r;
    EXPECT_EQ(r.send("WATCHDOG"), std::vector<std::string>{line("WATCHDOG,500")});
    EXPECT_EQ(r.send("WATCHDOG,1000"), std::vector<std::string>{line("ACK,WATCHDOG")});
    EXPECT_EQ(r.send("WATCHDOG"), std::vector<std::string>{line("WATCHDOG,1000")});
    EXPECT_EQ(r.send("WATCHDOG,10"), std::vector<std::string>{line("NAK,WATCHDOG,RNG")});
    EXPECT_EQ(r.send("WATCHDOG,1.5"), std::vector<std::string>{line("NAK,WATCHDOG,FMT")});
    EXPECT_EQ(r.send("WATCHDOG,0"), std::vector<std::string>{line("ACK,WATCHDOG")});
}

TEST(NmeaProtocol, VelocityCancelsPendingArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,90,0");
    (void) r.send("VEL,0,0");
    EXPECT_TRUE(r.advance(3000).empty());
}

TEST(NmeaProtocol, ClockReportsCurrentTimeNotTickTime)
{
    Rig r;
    r.proc.service(r.now);
    (void) r.port.takeLines();
    r.clock.t += 7;  // time passes between loop ticks
    const std::string f = frame("CLOCK");
    (void) r.proto.consume(reinterpret_cast<const uint8_t*>(f.data()), f.size());
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("CLOCK,1007")});
}

TEST(NmeaProtocol, PosIsStampedWithTheTickThatProducedIt)
{
    Rig r;
    r.proc.service(r.now);  // pose computed at t=1000
    r.clock.t += 7;
    const std::string f = frame("POS");
    (void) r.proto.consume(reinterpret_cast<const uint8_t*>(f.data()), f.size());
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("POS,0.0,0.0,0.0,0.0,D,1000")});
}

// ── Review findings (adversarial review 2026-10-01) ─────────────────────────

TEST(NmeaProtocolReview, BareEnableDuringVelocityDoesNotArmArrival)  // M1
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("VEL,10,0");
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.advance(1000), std::vector<std::string>{line("TIMEOUT,VEL")});  // no ARRIVED
}

TEST(NmeaProtocolReview, EnableAtPoseWhileEnabledAlwaysArrives)  // M1
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("ENABLE,0,0"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ARRIVED,0.0,0.0")});
}

TEST(NmeaProtocolReview, EnableFromDisabledNeverArrives)  // M1
{
    Rig r;
    (void) r.send("ENABLE,10,10");
    EXPECT_TRUE(r.advance(500).empty());
}

TEST(NmeaProtocolReview, StrayDollarBeforeBinaryIsHandedBackSilently)  // M2
{
    Rig r;
    const std::string in = std::string("$") + '\xFF';
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(res.consumed, 1u);  // 0xFF left for another protocol
    EXPECT_TRUE(r.port.takeLines().empty());
}

TEST(NmeaProtocolReview, GarbledRequestStillGetsAReply)  // N1
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
        EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,2,0.1.0")});
    }
}

TEST(NmeaProtocolReview, NonTextInsideAFrameHandsTheByteBackAfterReporting)  // N1 + M2
{
    Rig r;
    const std::string in = std::string("$PI") + '\xFF' + "NG";
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(res.consumed, 3u);
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("ERR,SYN")});
}

TEST(NmeaProtocolReview, OverflowHandsTheStreamBack)  // M2
{
    Rig r;
    const std::string in = "$" + std::string(100, 'A');
    const ::owlet::core::ConsumeResult res =
        r.proto.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_TRUE(res.released);
    EXPECT_EQ(r.port.takeLines(), std::vector<std::string>{line("ERR,LEN")});
}
