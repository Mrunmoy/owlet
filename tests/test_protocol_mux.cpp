#include "FakeProtocol.hpp"
#include "owlet/core/ProtocolMux.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using ::owlet::core::ConsumeResult;
using ::owlet::core::ProtocolMux;
using ::owlet::core::TickStatus;
using ::owlet::test::FakeProtocol;

namespace
{

// Feed a whole string through the mux, the way ControlLoop does: keep
// calling consume() until every byte has been taken.
void feed(::owlet::core::IProtocol& p, const std::string& s)
{
    const auto*  data = reinterpret_cast<const uint8_t*>(s.data());
    std::size_t  at   = 0;
    while (at < s.size())
    {
        const ConsumeResult r = p.consume(data + at, s.size() - at);
        at += (r.consumed != 0) ? r.consumed : 1;  // same rule as ControlLoop
    }
}

// Binary frames of fixed length starting with 0xFF (Pelco-D-like).
class FakeBinary final : public ::owlet::core::IProtocol
{
public:
    [[nodiscard]] bool claims(uint8_t byte, bool) const override { return byte == 0xFF; }
    [[nodiscard]] ConsumeResult consume(const uint8_t* data, std::size_t len) override
    {
        for (std::size_t i = 0; i < len; ++i)
        {
            current.push_back(data[i]);
            if (current.size() == 7)
            {
                ++frames;
                current.clear();
                return {i + 1, true};
            }
        }
        return {len, false};
    }
    void begin(uint32_t) override {}
    void onTick(uint32_t, const TickStatus&) override {}

    std::vector<uint8_t> current;
    int                  frames{0};
};

// Owns '$' lines but gives the stream back, unconsumed, at any byte >= 0x80.
class YieldingProtocol final : public ::owlet::core::IProtocol
{
public:
    [[nodiscard]] bool claims(uint8_t byte, bool) const override { return byte == '$'; }
    [[nodiscard]] ConsumeResult consume(const uint8_t* data, std::size_t len) override
    {
        for (std::size_t i = 0; i < len; ++i)
        {
            if (data[i] >= 0x80)
            {
                return {i, true};  // not mine: re-arbitrate this byte
            }
            if (data[i] == '\n')
            {
                return {i + 1, true};
            }
        }
        return {len, false};
    }
    void begin(uint32_t) override {}
    void onTick(uint32_t, const TickStatus&) override {}
};

// Claims everything and never consumes: must not hang the mux.
class StuckProtocol final : public ::owlet::core::IProtocol
{
public:
    [[nodiscard]] bool claims(uint8_t, bool) const override { return true; }
    [[nodiscard]] ConsumeResult consume(const uint8_t*, std::size_t) override { return {0, true}; }
    void begin(uint32_t) override {}
    void onTick(uint32_t, const TickStatus&) override {}
};

} // namespace

TEST(ProtocolMux, RoutesFrameToClaimingProtocol)
{
    FakeProtocol  dollar('$', true);
    FakeProtocol  hash('#', false);
    ProtocolMux<2> mux({&dollar, &hash});
    feed(mux, "$A\n#B\n");
    EXPECT_EQ(dollar.frames, std::vector<std::string>{"$A\n"});
    EXPECT_EQ(hash.frames, std::vector<std::string>{"#B\n"});
}

TEST(ProtocolMux, DropsUnclaimedLinesEntirely)
{
    FakeProtocol   hash('#', false);
    ProtocolMux<1> mux({&hash});
    // '#' inside the noise line must not be claimed: it isn't at line start.
    feed(mux, "ESP-ROM # boot\r\n#OK\n");
    EXPECT_EQ(hash.frames, std::vector<std::string>{"#OK\n"});
}

TEST(ProtocolMux, AnywhereProtocolClaimsMidLine)
{
    FakeProtocol   dollar('$', true);
    ProtocolMux<1> mux({&dollar});
    feed(mux, "garbage$PING\n");
    EXPECT_EQ(dollar.frames, std::vector<std::string>{"$PING\n"});
}

TEST(ProtocolMux, OwnershipPersistsAcrossChunks)
{
    FakeProtocol   dollar('$', true);
    FakeProtocol   hash('#', true);
    ProtocolMux<2> mux({&dollar, &hash});
    feed(mux, "$AB");
    feed(mux, "#CD\n");  // '#' arrives while '$' owns the stream
    EXPECT_EQ(dollar.frames, std::vector<std::string>{"$AB#CD\n"});
    EXPECT_TRUE(hash.frames.empty());
}

TEST(ProtocolMux, FirstClaimerInOrderWins)
{
    FakeProtocol   first('$', true);
    FakeProtocol   second('$', true);
    ProtocolMux<2> mux({&first, &second});
    feed(mux, "$X\n");
    EXPECT_EQ(first.frames.size(), 1u);
    EXPECT_TRUE(second.frames.empty());
}

TEST(ProtocolMux, LineStartIsRestoredAfterRelease)
{
    FakeProtocol   dollar('$', true);
    FakeProtocol   hash('#', false);
    ProtocolMux<2> mux({&dollar, &hash});
    feed(mux, "$A\n#B\n");  // '#' directly after a released frame is at line start
    EXPECT_EQ(hash.frames.size(), 1u);
}

TEST(ProtocolMux, OwnerGetsWholeChunkInOneCall)
{
    FakeProtocol   dollar('$', true);
    ProtocolMux<1> mux({&dollar});
    feed(mux, "$ABCDEFGHIJKLMNOPQRSTUVWXYZ\n");
    EXPECT_EQ(dollar.consumeCalls, 1);
}

TEST(ProtocolMux, HandlesOwnerThatConsumesPartially)
{
    FakeProtocol   dollar('$', true);
    dollar.maxPerCall = 3;
    ProtocolMux<1> mux({&dollar});
    feed(mux, "$ABCDEFG\n$H\n");
    EXPECT_EQ(dollar.frames, (std::vector<std::string>{"$ABCDEFG\n", "$H\n"}));
}

TEST(ProtocolMux, BeginAndTickFanOutToAll)
{
    FakeProtocol   a('$', true);
    FakeProtocol   b('#', true);
    ProtocolMux<2> mux({&a, &b});
    mux.begin(5);
    TickStatus st;
    st.busOk = false;
    mux.onTick(7, st);
    EXPECT_EQ(a.beginAt, std::vector<uint32_t>{5});
    EXPECT_EQ(b.beginAt, std::vector<uint32_t>{5});
    ASSERT_EQ(b.statuses.size(), 1u);
    EXPECT_FALSE(b.statuses[0].busOk);
    EXPECT_EQ(a.tickAt, std::vector<uint32_t>{7});
}

TEST(ProtocolMux, ClaimsWhatAnyChildClaims)
{
    FakeProtocol   dollar('$', true);
    FakeProtocol   hash('#', false);
    ProtocolMux<2> mux({&dollar, &hash});
    EXPECT_TRUE(mux.claims('$', false));
    EXPECT_TRUE(mux.claims('#', true));
    EXPECT_FALSE(mux.claims('#', false));
    EXPECT_FALSE(mux.claims('x', true));
}

TEST(ProtocolMux, NestedMuxIsAProtocol)
{
    FakeProtocol   dollar('$', true);
    FakeProtocol   hash('#', false);
    ProtocolMux<1> inner({&hash});
    ProtocolMux<2> outer({&dollar, &inner});
    feed(outer, "#A\n$B\n");
    EXPECT_EQ(hash.frames.size(), 1u);
    EXPECT_EQ(dollar.frames.size(), 1u);
}

TEST(ProtocolMux, OwnerCanReleaseWithoutConsumingToReArbitrate)  // M2
{
    YieldingProtocol text;
    FakeBinary       bin;
    ProtocolMux<2>   mux({&text, &bin});
    feed(mux, "$");  // a stray '$' grabs the stream at the end of one chunk...
    std::string frames;
    for (int f = 0; f < 3; ++f)
    {
        frames += std::string("\xFF\x01\x00\x04\x20\x00\x25", 7);
    }
    feed(mux, frames);  // ...so the next chunk opens with a byte the owner rejects
    EXPECT_EQ(bin.frames, 3);
}

TEST(ProtocolMux, FreshClaimThatConsumesNothingIsDroppedNotLooped)
{
    StuckProtocol  stuck;
    ProtocolMux<1> mux({&stuck});
    const std::string in = "abc";
    const ConsumeResult r = mux.consume(reinterpret_cast<const uint8_t*>(in.data()), in.size());
    EXPECT_EQ(r.consumed, in.size());
}

TEST(ProtocolMux, NestedMuxUsesTheCallersLineStartState)  // L2
{
    FakeProtocol   lineOnly('X', false);  // claims 'X' only at line start
    FakeProtocol   anywhere('X', true);
    ProtocolMux<2> inner({&lineOnly, &anywhere});
    ProtocolMux<1> outer({&inner});
    feed(outer, "noise X1\n");  // the 'X' is mid-line: only `anywhere` may take it
    EXPECT_TRUE(lineOnly.frames.empty());
    EXPECT_EQ(anywhere.frames, std::vector<std::string>{"X1\n"});
}
