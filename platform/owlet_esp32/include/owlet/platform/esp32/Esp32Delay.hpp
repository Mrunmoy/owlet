#pragma once

#include "owlet/hal/IDelayProvider.hpp"

#include <cstdint>

namespace owlet::platform::esp32
{

/// IDelayProvider backed by ESP-IDF timing primitives. Short delays (< 1 ms)
/// use esp_rom_delay_us for deterministic busy-waiting; longer delays fall
/// through to vTaskDelay so the scheduler can run other tasks.
class Esp32Delay final : public ::owlet::hal::IDelayProvider
{
public:
    void delayUs(uint32_t us) override;
    void delayMs(uint32_t ms) override;
};

} // namespace owlet::platform::esp32
