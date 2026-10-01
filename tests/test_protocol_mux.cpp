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
        ASSERT_GT(r.consumed, 0u);
        at += r.consumed;
    }
}

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
