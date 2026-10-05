#pragma once

#include "owlet/hal/IInterruptLock.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

/// IInterruptLock on the RP2040: masks interrupts on the calling core and
/// puts them back as they were. The core has no atomic compare-and-swap, so
/// this is how a few instructions are made safe against an interrupt.
class PicoInterruptLock final : public ::owlet::hal::IInterruptLock
{
public:
    [[nodiscard]] uint32_t lock() override;
    void unlock(uint32_t saved) override;
};

} // namespace owlet::platform::pico
