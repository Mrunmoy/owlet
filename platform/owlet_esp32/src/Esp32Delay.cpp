#include "owlet/platform/esp32/Esp32Delay.hpp"

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace owlet::platform::esp32
{

namespace
{
constexpr uint32_t kBusyWaitCeilUs = 1000U;
}

void Esp32Delay::delayUs(uint32_t us)
{
    if (us == 0U) return;
    if (us <= kBusyWaitCeilUs)
    {
        esp_rom_delay_us(us);
        return;
    }
    // Long µs delays — round up to the nearest millisecond tick.
    const uint32_t ms = (us + 999U) / 1000U;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void Esp32Delay::delayMs(uint32_t ms)
{
    if (ms == 0U) return;
    vTaskDelay(pdMS_TO_TICKS(ms));
}

} // namespace owlet::platform::esp32
