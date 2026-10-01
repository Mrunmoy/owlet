#include "owlet/platform/esp32/Esp32Clock.hpp"

#include "esp_timer.h"

namespace owlet::platform::esp32
{

uint32_t Esp32Clock::nowMs() const
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

} // namespace owlet::platform::esp32
