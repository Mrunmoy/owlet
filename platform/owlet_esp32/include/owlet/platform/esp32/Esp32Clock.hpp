#pragma once

#include "owlet/hal/IClock.hpp"

#include <cstdint>

namespace owlet::platform::esp32
{

/// IClock on esp_timer (microseconds since boot), truncated to 32-bit ms.
class Esp32Clock final : public ::owlet::hal::IClock
{
public:
    [[nodiscard]] uint32_t nowMs() const override;
};

} // namespace owlet::platform::esp32
