#include "FakeSerialPort.hpp"
#include "FakeServoChannel.hpp"
#include "owlet/core/CommandProcessor.hpp"
#include "owlet/protocol/LineWriter.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using ::owlet::Servo;
using ::owlet::core::CommandProcessor;
using ::owlet::core::PanTilt;
using ::owlet::protocol::LineWriter;
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
    CommandProcessor proc{port, head, "0.1.0"};
    uint32_t         now{1000};

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
};

} // namespace

TEST(CommandProcessor, AnnouncesReadyOnBegin)
{
    FakeServoChannel a;
    FakeServoChannel b;
    Servo            pan{a, Servo::Calibration{}};
    Servo            tilt{b, Servo::Calibration{}};
    PanTilt          head{pan, tilt};
    FakeSerialPort   port;
    CommandProcessor proc{port, head, "0.1.0"};
    proc.begin(0);
    EXPECT_EQ(port.takeLines(), std::vector<std::string>{line("READY,1,0.1.0")});
}

TEST(CommandProcessor, PingReturnsVersions)
{
    Rig r;
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,1,0.1.0")});
}

TEST(CommandProcessor, PosReportsDisabledAtBoot)
{
    Rig r;
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{line("POS,0.0,0.0,0.0,0.0,D")});
}

TEST(CommandProcessor, MoveWhileDisabledIsNakDis)
{
    Rig r;
    EXPECT_EQ(r.send("MOVE,10,10"), std::vector<std::string>{line("NAK,MOVE,DIS")});
    EXPECT_TRUE(r.panCh.writes.empty());
}

TEST(CommandProcessor, EnableMoveArriveSequence)
{
    Rig r;
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("ACK,ENABLE")});
    EXPECT_EQ(r.send("SPEED,100"), std::vector<std::string>{line("ACK,SPEED")});
    EXPECT_EQ(r.send("MOVE,30,-15"), std::vector<std::string>{line("ACK,MOVE")});

    const std::vector<std::string> mid = r.advance(100);
    EXPECT_TRUE(mid.empty());
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{line("POS,10.0,-10.0,30.0,-15.0,M")});

    const std::vector<std::string> done = r.advance(300);
    EXPECT_EQ(done, std::vector<std::string>{line("ARRIVED,30.0,-15.0")});
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{line("POS,30.0,-15.0,30.0,-15.0,H")});
}

TEST(CommandProcessor, ArrivedFiresEvenForZeroLengthMove)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("HOME"), std::vector<std::string>{line("ACK,HOME")});
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ARRIVED,0.0,0.0")});
    EXPECT_TRUE(r.advance(100).empty());
}

TEST(CommandProcessor, StopCancelsPendingArrival)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,90,0");
    (void) r.advance(50);
    EXPECT_EQ(r.send("STOP"), std::vector<std::string>{line("ACK,STOP")});
    EXPECT_TRUE(r.advance(2000).empty());
}

TEST(CommandProcessor, RangeAndFormatErrors)
{
    Rig r;
    (void) r.send("ENABLE");
    EXPECT_EQ(r.send("MOVE,91,0"), std::vector<std::string>{line("NAK,MOVE,RNG")});
    EXPECT_EQ(r.send("MOVE,1"), std::vector<std::string>{line("NAK,MOVE,FMT")});
    EXPECT_EQ(r.send("SPEED,0"), std::vector<std::string>{line("NAK,SPEED,RNG")});
    EXPECT_EQ(r.send("STREAM,51"), std::vector<std::string>{line("NAK,STREAM,RNG")});
    EXPECT_EQ(r.send("FLY"), std::vector<std::string>{line("ERR,UNK")});
}

TEST(CommandProcessor, FrameLevelErrors)
{
    Rig r;
    r.port.rx += "$PING*11\r\n";
    r.port.rx += "$PING*1\r\n";
    r.port.rx += "$" + std::string(120, 'A') + "\r\n";
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("ERR,CHK"), line("ERR,SYN"), line("ERR,LEN")}));
}

TEST(CommandProcessor, IgnoresBootNoiseBeforeFirstFrame)
{
    Rig r;
    r.port.rx += "ESP-ROM:esp32s3\r\nbuild: Mar 27 2021\r\n";
    EXPECT_EQ(r.send("PING"), std::vector<std::string>{line("PONG,1,0.1.0")});
}

TEST(CommandProcessor, SeveralRequestsInOneReadAnsweredInOrder)
{
    Rig r;
    r.port.rx += frame("PING") + frame("LIMITS") + frame("SPEED");
    r.proc.service(r.now);
    EXPECT_EQ(r.port.takeLines(),
              (std::vector<std::string>{line("PONG,1,0.1.0"),
                                        line("LIMITS,-90.0,90.0,-90.0,90.0"),
                                        line("SPEED,90.0")}));
}

TEST(CommandProcessor, StreamEmitsPosAtRequestedRate)
{
    Rig r;
    EXPECT_EQ(r.send("STREAM,10"), std::vector<std::string>{line("ACK,STREAM")});
    const std::vector<std::string> lines = r.advance(1000);
    EXPECT_EQ(lines.size(), 10u);
    for (const std::string& l : lines)
    {
        EXPECT_EQ(l, line("POS,0.0,0.0,0.0,0.0,D"));
    }
    EXPECT_EQ(r.send("STREAM,0"), std::vector<std::string>{line("ACK,STREAM")});
    EXPECT_TRUE(r.advance(1000).empty());
}

TEST(CommandProcessor, DisableGoesLimpAndReportsD)
{
    Rig r;
    (void) r.send("ENABLE,20,10");
    EXPECT_EQ(r.send("DISABLE"), std::vector<std::string>{line("ACK,DISABLE")});
    EXPECT_TRUE(r.panCh.idle);
    EXPECT_EQ(r.send("POS"), std::vector<std::string>{line("POS,20.0,10.0,20.0,10.0,D")});
}

TEST(CommandProcessor, EnableBusFailureIsNakBus)
{
    Rig r;
    r.panCh.failNext = true;
    EXPECT_EQ(r.send("ENABLE"), std::vector<std::string>{line("NAK,ENABLE,BUS")});
}

TEST(CommandProcessor, SlewBusFailureReportsErrBusOnceUntilRecovered)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("MOVE,10,0");
    r.panCh.failNext = true;
    EXPECT_EQ(r.advance(10), std::vector<std::string>{line("ERR,BUS")});
    const std::vector<std::string> rest = r.advance(1000);
    EXPECT_EQ(rest, std::vector<std::string>{line("ARRIVED,10.0,0.0")});
}

TEST(CommandProcessor, LargeTimeGapIsCappedSoHeadDoesNotJump)
{
    Rig r;
    (void) r.send("ENABLE");
    (void) r.send("SPEED,100");
    (void) r.send("MOVE,90,0");
    r.now += 5000;  // e.g. the loop was starved
    r.proc.service(r.now);
    (void) r.port.takeLines();
    EXPECT_LE(r.head.panDeg(), 100.0f * CommandProcessor::kMaxTickMs / 1000.0f + 1e-3f);
}
