#include "owlet/nmea/Checksum.hpp"
#include "owlet/nmea/LineFramer.hpp"
#include "owlet/nmea/LineWriter.hpp"
#include "owlet/nmea/Request.hpp"
#include "owlet/nmea/Sentence.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <string_view>
#include <vector>

using namespace ::owlet::nmea;

namespace
{

// Feed a whole string and collect every complete frame / overflow event.
struct FramerHarness
{
    LineFramer               framer;
    std::vector<std::string> frames;
    int                      overflows{0};

    void feed(std::string_view s)
    {
        for (char c : s)
        {
            switch (framer.push(static_cast<uint8_t>(c)))
            {
            case LineFramer::Event::Frame:
                frames.emplace_back(framer.frame());
                break;
            case LineFramer::Event::Overflow:
                ++overflows;
                break;
            case LineFramer::Event::None:
                break;
            }
        }
    }
};

Request parse(std::string_view frame)
{
    Sentence s;
    EXPECT_EQ(decodeSentence(frame, s), FrameStatus::Ok) << frame;
    Request r;
    r.status = parseRequest(s, r);
    return r;
}

std::string encode(std::string_view body)
{
    LineWriter w;
    w.beginRaw(body);
    return std::string(w.finish());
}

} // namespace

// ── Checksum ────────────────────────────────────────────────────────────────

TEST(Checksum, XorOfBody)
{
    EXPECT_EQ(checksum("PING"), 0x10);
    EXPECT_EQ(checksum("MOVE,30,-15"), 0x3B);
    EXPECT_EQ(checksum(""), 0x00);
}

// ── LineFramer ──────────────────────────────────────────────────────────────

TEST(LineFramer, ExtractsFrameTerminatedByCrLf)
{
    FramerHarness h;
    h.feed("$PING*10\r\n");
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0], "$PING*10");
}

TEST(LineFramer, AcceptsBareLf)
{
    FramerHarness h;
    h.feed("$PING*10\n");
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0], "$PING*10");
}

TEST(LineFramer, IgnoresBytesOutsideFrames)
{
    FramerHarness h;
    h.feed("ESP-ROM:esp32s3-20210327\r\nrst:0x1\r\n$POS*4C\r\n");
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0], "$POS*4C");
}

TEST(LineFramer, DollarRestartsPartialFrame)
{
    FramerHarness h;
    h.feed("$MOV$PING*10\r\n");
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0], "$PING*10");
}

TEST(LineFramer, HandlesSplitDelivery)
{
    FramerHarness h;
    h.feed("$MO");
    h.feed("VE,30,-1");
    EXPECT_TRUE(h.frames.empty());
    h.feed("5*3B\r\n$PI");
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0], "$MOVE,30,-15*3B");
}

TEST(LineFramer, OverflowReportedOnceAndRecovers)
{
    FramerHarness h;
    h.feed("$" + std::string(200, 'A') + "\r\n");
    EXPECT_EQ(h.overflows, 1);
    EXPECT_TRUE(h.frames.empty());
    h.feed("$PING*10\r\n");
    ASSERT_EQ(h.frames.size(), 1u);
}

TEST(LineFramer, MaxLengthFrameFits)
{
    // 82 bytes on the wire = 80 bytes of frame + CRLF.
    const std::string body(80 - 4, 'A');  // "$" + body + "*HH" = 80
    FramerHarness h;
    h.feed("$" + body + "*00\r\n");
    EXPECT_EQ(h.overflows, 0);
    ASSERT_EQ(h.frames.size(), 1u);
    EXPECT_EQ(h.frames[0].size(), 80u);
}

TEST(LineFramer, OneByteOverMaxOverflows)
{
    const std::string body(80 - 3, 'A');
    FramerHarness h;
    h.feed("$" + body + "*00\r\n");
    EXPECT_EQ(h.overflows, 1);
    EXPECT_TRUE(h.frames.empty());
}

TEST(LineFramer, EmptyLinesAreIgnored)
{
    FramerHarness h;
    h.feed("\r\n\r\n$\r\n");
    EXPECT_TRUE(h.frames.empty());
    EXPECT_EQ(h.overflows, 0);
}

// ── Sentence decoding ───────────────────────────────────────────────────────

TEST(Sentence, SplitsNameAndFields)
{
    Sentence s;
    ASSERT_EQ(decodeSentence("$MOVE,30,-15*3B", s), FrameStatus::Ok);
    EXPECT_EQ(s.name, "MOVE");
    ASSERT_EQ(s.fieldCount, 2u);
    EXPECT_EQ(s.fields[0], "30");
    EXPECT_EQ(s.fields[1], "-15");
}

TEST(Sentence, KeepsEmptyFields)
{
    Sentence s;
    const std::string f = encode("NAK,,CHK");
    ASSERT_EQ(decodeSentence(std::string_view(f).substr(0, f.size() - 2), s), FrameStatus::Ok);
    ASSERT_EQ(s.fieldCount, 2u);
    EXPECT_EQ(s.fields[0], "");
    EXPECT_EQ(s.fields[1], "CHK");
}

TEST(Sentence, TooManyFieldsIsSyntaxError)
{
    const std::string f = encode("MOVE,1,2,3,4,5,6,7,8,9");
    Sentence s;
    EXPECT_EQ(decodeSentence(std::string_view(f).substr(0, f.size() - 2), s),
              FrameStatus::Syntax);
}

TEST(Sentence, LowerCaseNameIsSyntaxError)
{
    const std::string f = encode("ping");
    Sentence s;
    EXPECT_EQ(decodeSentence(std::string_view(f).substr(0, f.size() - 2), s),
              FrameStatus::Syntax);
}

TEST(Sentence, SharedVectors)
{
    std::ifstream in(OWLET_PROTOCOL_VECTORS);
    ASSERT_TRUE(in.is_open()) << OWLET_PROTOCOL_VECTORS;
    std::string line;
    int         checked = 0;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        const std::size_t sp     = line.find(' ');
        const std::string expect = line.substr(0, sp);
        const std::string frame  = line.substr(line.find_first_not_of(' ', sp));
        Sentence          s;
        const FrameStatus got = decodeSentence(frame, s);
        const FrameStatus want = expect == "OK"    ? FrameStatus::Ok
                                 : expect == "CHK" ? FrameStatus::Checksum
                                                   : FrameStatus::Syntax;
        EXPECT_EQ(got, want) << line;
        ++checked;
    }
    EXPECT_GT(checked, 20);
}

// ── Request parsing ─────────────────────────────────────────────────────────

TEST(Request, ParsesEveryCommand)
{
    EXPECT_EQ(parse("$PING*10").command, Command::Ping);
    EXPECT_EQ(parse("$DISABLE*54").command, Command::Disable);
    EXPECT_EQ(parse("$HOME*0F").command, Command::Home);
    EXPECT_EQ(parse("$STOP*18").command, Command::Stop);
    EXPECT_EQ(parse("$POS*4C").command, Command::Pos);
    EXPECT_EQ(parse("$LIMITS*06").command, Command::Limits);
}

TEST(Request, MoveCarriesTwoAngles)
{
    const Request r = parse("$MOVE,12.5,-0.25*3D");
    EXPECT_EQ(r.status, RequestStatus::Ok);
    EXPECT_EQ(r.command, Command::Move);
    EXPECT_EQ(r.argCount, 2u);
    EXPECT_FLOAT_EQ(r.args[0], 12.5f);
    EXPECT_FLOAT_EQ(r.args[1], -0.25f);
}

TEST(Request, EnableWithAndWithoutPose)
{
    const Request bare = parse("$ENABLE*01");
    EXPECT_EQ(bare.status, RequestStatus::Ok);
    EXPECT_EQ(bare.argCount, 0u);
    const Request pose = parse("$ENABLE,0,0*01");
    EXPECT_EQ(pose.status, RequestStatus::Ok);
    EXPECT_EQ(pose.argCount, 2u);
}

TEST(Request, SpeedQueryAndSet)
{
    EXPECT_EQ(parse("$SPEED*47").argCount, 0u);
    const Request set = parse("$SPEED,120*58");
    EXPECT_EQ(set.argCount, 1u);
    EXPECT_FLOAT_EQ(set.args[0], 120.0f);
}

TEST(Request, StreamRequiresInteger)
{
    const Request ok = parse("$STREAM,10*31");
    EXPECT_EQ(ok.status, RequestStatus::Ok);
    EXPECT_FLOAT_EQ(ok.args[0], 10.0f);
    const std::string bad = encode("STREAM,2.5");
    EXPECT_EQ(parse(std::string_view(bad).substr(0, bad.size() - 2)).status,
              RequestStatus::BadFormat);
}

TEST(Request, WrongArgCountIsBadFormat)
{
    for (const char* body : {"MOVE,1", "MOVE", "MOVE,1,2,3", "PING,1", "ENABLE,1", "NUDGE"})
    {
        const std::string f = encode(body);
        EXPECT_EQ(parse(std::string_view(f).substr(0, f.size() - 2)).status,
                  RequestStatus::BadFormat)
            << body;
    }
}

TEST(Request, MalformedNumbersAreBadFormat)
{
    for (const char* body :
         {"MOVE,1e3,0", "MOVE,nan,0", "MOVE,,0", "MOVE,1.,0", "MOVE,.5,0", "MOVE,1.234,0",
          "MOVE,--1,0", "MOVE,1-,0", "MOVE,12345,0", "MOVE, 1,0"})
    {
        const std::string f = encode(body);
        EXPECT_EQ(parse(std::string_view(f).substr(0, f.size() - 2)).status,
                  RequestStatus::BadFormat)
            << body;
    }
}

TEST(Request, UnknownCommand)
{
    const std::string f = encode("FLY,1");
    const Request     r = parse(std::string_view(f).substr(0, f.size() - 2));
    EXPECT_EQ(r.status, RequestStatus::Unknown);
}

TEST(Request, VelocityCarriesTwoSignedRates)
{
    const std::string f = encode("VEL,-12.5,30");
    const Request     r = parse(std::string_view(f).substr(0, f.size() - 2));
    EXPECT_EQ(r.status, RequestStatus::Ok);
    EXPECT_EQ(r.command, Command::Vel);
    EXPECT_EQ(r.argCount, 2u);
    EXPECT_FLOAT_EQ(r.args[0], -12.5f);
    EXPECT_FLOAT_EQ(r.args[1], 30.0f);
}

TEST(Request, WatchdogQueryOrIntegerSet)
{
    for (const char* body : {"WATCHDOG", "WATCHDOG,250"})
    {
        const std::string f = encode(body);
        const Request     r = parse(std::string_view(f).substr(0, f.size() - 2));
        EXPECT_EQ(r.status, RequestStatus::Ok) << body;
        EXPECT_EQ(r.command, Command::Watchdog);
    }
    for (const char* body : {"WATCHDOG,2.5", "WATCHDOG,-1", "WATCHDOG,1,2"})
    {
        const std::string f = encode(body);
        EXPECT_EQ(parse(std::string_view(f).substr(0, f.size() - 2)).status,
                  RequestStatus::BadFormat)
            << body;
    }
}

TEST(Request, ClockTakesNoArguments)
{
    const std::string ok = encode("CLOCK");
    EXPECT_EQ(parse(std::string_view(ok).substr(0, ok.size() - 2)).command, Command::Clock);
    const std::string bad = encode("CLOCK,1");
    EXPECT_EQ(parse(std::string_view(bad).substr(0, bad.size() - 2)).status,
              RequestStatus::BadFormat);
}

TEST(Request, CapsTakesNoArguments)
{
    const std::string ok = encode("CAPS");
    EXPECT_EQ(parse(std::string_view(ok).substr(0, ok.size() - 2)).command, Command::Caps);
    const std::string bad = encode("CAPS,1");
    EXPECT_EQ(parse(std::string_view(bad).substr(0, bad.size() - 2)).status,
              RequestStatus::BadFormat);
}

TEST(Request, CalQueriesAndActionsCarryAToken)
{
    for (const char* t : {"PAN", "TILT", "SAVE", "RESET"})
    {
        const std::string f = encode(std::string("CAL,") + t);
        const Request     r = parse(std::string_view(f).substr(0, f.size() - 2));
        EXPECT_EQ(r.status, RequestStatus::Ok) << t;
        EXPECT_EQ(r.command, Command::Cal);
        EXPECT_EQ(r.token, t);
        EXPECT_EQ(r.argCount, 0u);
    }
}

TEST(Request, CalSetCarriesAxisAndFiveNumbers)
{
    const std::string f = encode("CAL,TILT,2500,1500,500,-90,87.5");
    const Request     r = parse(std::string_view(f).substr(0, f.size() - 2));
    ASSERT_EQ(r.status, RequestStatus::Ok);
    EXPECT_EQ(r.token, "TILT");
    ASSERT_EQ(r.argCount, 5u);
    EXPECT_FLOAT_EQ(r.args[0], 2500.0f);
    EXPECT_FLOAT_EQ(r.args[1], 1500.0f);
    EXPECT_FLOAT_EQ(r.args[2], 500.0f);
    EXPECT_FLOAT_EQ(r.args[3], -90.0f);
    EXPECT_FLOAT_EQ(r.args[4], 87.5f);
}

TEST(Request, MalformedCalIsBadFormat)
{
    for (const char* body : {"CAL", "CAL,FOO", "CAL,SAVE,1", "CAL,PAN,500,1500,2500,-90",
                             "CAL,PAN,500.5,1500,2500,-90,90", "CAL,PAN,-500,1500,2500,-90,90",
                             "CAL,SAVE,500,1500,2500,-90,90", "CAL,PAN,500,1500,2500,-90,90,1"})
    {
        const std::string f = encode(body);
        EXPECT_EQ(parse(std::string_view(f).substr(0, f.size() - 2)).status,
                  RequestStatus::BadFormat)
            << body;
    }
}

TEST(Request, CommandNamesRoundTrip)
{
    EXPECT_EQ(commandName(Command::Move), "MOVE");
    EXPECT_EQ(commandName(Command::Stream), "STREAM");
    EXPECT_EQ(commandName(Command::Limits), "LIMITS");
    EXPECT_EQ(commandName(Command::Vel), "VEL");
    EXPECT_EQ(commandName(Command::Watchdog), "WATCHDOG");
    EXPECT_EQ(commandName(Command::Clock), "CLOCK");
    EXPECT_EQ(commandName(Command::Caps), "CAPS");
    EXPECT_EQ(commandName(Command::Cal), "CAL");
}

// ── LineWriter ──────────────────────────────────────────────────────────────

TEST(LineWriter, BuildsChecksummedLine)
{
    LineWriter w;
    w.begin("ACK");
    w.field("MOVE");
    EXPECT_EQ(w.finish(), "$ACK,MOVE*74\r\n");
}

TEST(LineWriter, FormatsOneDecimalFixedPoint)
{
    LineWriter w;
    w.begin("POS");
    w.fixed1(12.5f);
    w.fixed1(-3.0f);
    w.fixed1(30.04f);
    w.fixed1(-15.05f);
    w.field("M");
    const std::string_view line = w.finish();
    EXPECT_EQ(line.substr(0, line.find('*')), "$POS,12.5,-3.0,30.0,-15.1,M");
}

TEST(LineWriter, NegativeZeroPrintsAsZero)
{
    LineWriter w;
    w.begin("POS");
    w.fixed1(-0.01f);
    const std::string_view line = w.finish();
    EXPECT_EQ(line.substr(0, line.find('*')), "$POS,0.0");
}

TEST(LineWriter, FormatsUnsigned)
{
    LineWriter w;
    w.begin("SPEED");
    w.uint(120);
    EXPECT_EQ(w.finish(), "$SPEED,120*58\r\n");
}

TEST(LineWriter, OutputDecodesBack)
{
    LineWriter w;
    w.begin("ARRIVED");
    w.fixed1(30.0f);
    w.fixed1(-15.0f);
    const std::string_view line = w.finish();
    Sentence               s;
    EXPECT_EQ(decodeSentence(line.substr(0, line.size() - 2), s), FrameStatus::Ok);
    EXPECT_EQ(line, "$ARRIVED,30.0,-15.0*75\r\n");
}

TEST(LineWriter, FormatsFullUint32Range)
{
    LineWriter w;
    w.begin("CLOCK");
    w.uint(4294967295u);
    const std::string_view line = w.finish();
    EXPECT_EQ(line.substr(0, line.find('*')), "$CLOCK,4294967295");
}

TEST(LineWriter, OverlongLineIsTruncatedSafely)
{
    LineWriter w;
    w.begin("X");
    for (int i = 0; i < 100; ++i)
    {
        w.field("ABCDEFGH");
    }
    const std::string_view line = w.finish();
    EXPECT_LE(line.size(), LineWriter::kMaxLine);
    EXPECT_TRUE(w.overflowed());
}
