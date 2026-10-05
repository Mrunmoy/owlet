#include "owlet/platform/pico/PicoDelay.hpp"

#include "pico/time.h"

namespace owlet::platform::pico
{

void PicoDelay::delayUs(uint32_t us)
{
    sleep_us(us);
}

void PicoDelay::delayMs(uint32_t ms)
{
    sleep_ms(ms);
}

} // namespace owlet::platform::pico
