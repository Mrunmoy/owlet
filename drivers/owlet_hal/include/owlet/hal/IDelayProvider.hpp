#pragma once

#include <cstdint>

namespace owlet::hal
{

/// Blocking-delay abstraction. On the Pico this wraps sleep_us / sleep_ms;
/// host tests usually stub it with a no-op or a fake-clock
/// recorder.
class IDelayProvider
{
public:
    virtual ~IDelayProvider() = default;

    virtual void delayUs(uint32_t us) = 0;
    virtual void delayMs(uint32_t ms) = 0;
};

} // namespace owlet::hal
