#pragma once

#include "owlet/hal/IStepGenerator.hpp"

#include <cstdint>
#include <vector>

namespace owlet::test
{

// Records every burst. `busy` makes emit() refuse, as a full queue would.
class FakeStepGenerator final : public ::owlet::hal::IStepGenerator
{
public:
    struct Burst
    {
        int32_t steps;
        uint32_t durationUs;
    };

    [[nodiscard]] bool emit(int32_t steps, uint32_t durationUs) override
    {
        if (busy)
        {
            ++refused;
            return false;
        }
        bursts.push_back(Burst{steps, durationUs});
        total += steps;
        return true;
    }

    std::vector<Burst> bursts;
    int64_t total{0}; // net steps emitted, signed
    int refused{0};
    bool busy{false};
};

} // namespace owlet::test
