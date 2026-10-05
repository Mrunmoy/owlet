#pragma once

#include "owlet/hal/IDelayProvider.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

// Blocking delays on the pico-sdk sleep functions.
class PicoDelay final : public ::owlet::hal::IDelayProvider
{
public:
    void delayUs(uint32_t us) override;
    void delayMs(uint32_t ms) override;
};

} // namespace owlet::platform::pico
