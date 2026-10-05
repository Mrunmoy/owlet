#pragma once

#include "owlet/hal/IClock.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

// IClock on the RP2040's 64-bit microsecond timer, truncated to 32-bit ms.
class PicoClock final : public ::owlet::hal::IClock
{
public:
    [[nodiscard]] uint32_t nowMs() const override;
};

} // namespace owlet::platform::pico
