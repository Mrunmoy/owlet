#pragma once

#include <cstdint>

namespace owlet::hal
{

// Monotonic millisecond clock. Wraps after 2^32 ms (~49.7 days); consumers
// must compare times with unsigned or signed-difference arithmetic.
class IClock
{
public:
    virtual ~IClock() = default;

    [[nodiscard]] virtual uint32_t nowMs() const = 0;
};

} // namespace owlet::hal
