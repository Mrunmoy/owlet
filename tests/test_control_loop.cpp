#include "FakeProtocol.hpp"
#include "FakeSerialPort.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/ITickable.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using ::owlet::core::ControlLoop;
using ::owlet::core::TickStatus;
using ::owlet::test::FakeProtocol;
using ::owlet::test::FakeSerialPort;

namespace
{

class FakeTicker final : public ::owlet::core::ITickable
{
public:
    [[nodiscard]] TickStatus tick(uint32_t dtMs) override
    {
        dts.push_back(dtMs);
        if (log != nullptr) log->push_back("tick");
        return next;
    }

    std::vector<uint32_t>     dts;
    TickStatus                next{};
    std::vector<std::string>* log{nullptr};
};

// Protocol that logs when it receives bytes, to check ordering.
class LoggingProtocol final : public ::owlet::core::IProtocol
{
public:
    [[nodiscard]] bool claims(uint8_t, bool) const override { return true; }
    [[nodiscard]] ::owlet::core::ConsumeResult consume(const uint8_t* data,
                                                      std::size_t    len) override
    {
        log->push_back("consume");
        received.append(reinterpret_cast<const char*>(data), len);
        return {len, false};
    }
    void begin(uint32_t) override {}
    void onTick(uint32_t, const TickStatus&) override { log->push_back("onTick"); }

    std::vector<std::string>* log{nullptr};
    std::string               received;
};

} // namespace

TEST(ControlLoop, BeginForwardsTimeToProtocol)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    ControlLoop    loop(port, ticker, proto);
    loop.begin(1234);
    EXPECT_EQ(proto.beginAt, std::vector<uint32_t>{1234});
}

TEST(ControlLoop, TicksWithElapsedTime)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    ControlLoop    loop(port, ticker, proto);
    loop.begin(1000);
    loop.service(1010);
    loop.service(1025);
    EXPECT_EQ(ticker.dts, (std::vector<uint32_t>{10, 15}));
    EXPECT_EQ(proto.tickAt, (std::vector<uint32_t>{1010, 1025}));
}

TEST(ControlLoop, CapsLargeTimeGaps)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    ControlLoop    loop(port, ticker, proto);
    loop.begin(0);
    loop.service(5000);
    EXPECT_EQ(ticker.dts, std::vector<uint32_t>{ControlLoop::kMaxTickMs});
}

TEST(ControlLoop, SurvivesMillisWraparound)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    ControlLoop    loop(port, ticker, proto);
    loop.begin(0xFFFFFFF0u);
    loop.service(0x00000005u);
    EXPECT_EQ(ticker.dts, std::vector<uint32_t>{0x15u});
}

TEST(ControlLoop, PassesTickStatusToProtocol)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    ControlLoop    loop(port, ticker, proto);
    loop.begin(0);
    ticker.next.busOk = false;
    loop.service(10);
    ASSERT_EQ(proto.statuses.size(), 1u);
    EXPECT_FALSE(proto.statuses[0].busOk);
}

TEST(ControlLoop, TicksMotionBeforeReadingInput)
{
    std::vector<std::string> log;
    FakeSerialPort           port;
    FakeTicker               ticker;
    ticker.log = &log;
    LoggingProtocol proto;
    proto.log = &log;
    ControlLoop loop(port, ticker, proto);
    loop.begin(0);
    port.rx = "x";
    loop.service(10);
    EXPECT_EQ(log, (std::vector<std::string>{"tick", "onTick", "consume"}));
}

TEST(ControlLoop, DrainsAllPendingInputAcrossChunks)
{
    std::vector<std::string> log;
    FakeSerialPort           port;
    FakeTicker               ticker;
    LoggingProtocol          proto;
    proto.log = &log;
    ControlLoop loop(port, ticker, proto);
    loop.begin(0);
    port.rx = std::string(3 * ControlLoop::kReadChunk + 7, 'z');
    loop.service(10);
    EXPECT_EQ(proto.received.size(), port.rx.size());
}

TEST(ControlLoop, RetriesPartialConsumeWithoutLosingBytes)
{
    FakeSerialPort port;
    FakeTicker     ticker;
    FakeProtocol   proto('$', true);
    proto.maxPerCall = 2;
    ControlLoop loop(port, ticker, proto);
    loop.begin(0);
    port.rx = "$ABCDEFGH\n";
    loop.service(10);
    EXPECT_EQ(proto.frames, std::vector<std::string>{"$ABCDEFGH\n"});
}
