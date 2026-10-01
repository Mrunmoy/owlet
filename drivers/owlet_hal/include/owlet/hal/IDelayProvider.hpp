#pragma once

#include <cstdint>

namespace owlet::hal
{

// Blocking-delay abstraction. On ESP32 this wraps esp_rom_delay_us /
// vTaskDelay; host tests usually stub it with a no-op or a fake-clock
// recorder.
class IDelayProvider
{
public:
    virtual ~IDelayProvider() = default;

    virtual void delayUs(uint32_t us) = 0;
    virtual void delayMs(uint32_t ms) = 0;
};

} // namespace owlet::hal
