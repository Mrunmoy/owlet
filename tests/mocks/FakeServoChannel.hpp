#pragma once

#include "owlet/hal/IServoChannel.hpp"

#include <cstdint>
#include <vector>

namespace owlet::test
{

// Records every pulse written so tests can assert on the slew profile.
class FakeServoChannel final : public ::owlet::hal::IServoChannel
{
public:
    [[nodiscard]] bool setPulseUs(uint16_t pulseUs) override
    {
        if (failNext)
        {
            failNext = false;
            return false;
        }
        writes.push_back(pulseUs);
        lastPulseUs = pulseUs;
        idle        = false;
        return true;
    }

    [[nodiscard]] bool setIdle() override
    {
        if (failNext)
        {
            failNext = false;
            return false;
        }
        idle = true;
        ++idleCalls;
        return true;
    }

    std::vector<uint16_t> writes;
    uint16_t              lastPulseUs{0};
    bool                  idle{true};
    int                   idleCalls{0};
    bool                  failNext{false};
};

} // namespace owlet::test
