#pragma once

#include "owlet/hal/IClock.hpp"

#include <cstdint>

namespace owlet::test
{

class FakeClock final : public ::owlet::hal::IClock
{
public:
    explicit FakeClock(uint32_t start = 0) : t(start) {}

    [[nodiscard]] uint32_t nowMs() const override
    {
        return t;
    }

    uint32_t t;
};

} // namespace owlet::test
