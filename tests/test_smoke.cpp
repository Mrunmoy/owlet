// Smoke test: proves the host build -> GTest pipeline is wired up.
// Kept as the canary that GTest + FetchContent still work.

#include <gtest/gtest.h>

TEST(Smoke, HostBuildWorks)
{
    EXPECT_EQ(2 + 2, 4);
}
