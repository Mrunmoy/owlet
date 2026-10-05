#include "owlet/platform/pico/PicoClock.hpp"

#include "pico/time.h"

namespace owlet::platform::pico
{

uint32_t PicoClock::nowMs() const
{
    return to_ms_since_boot(get_absolute_time());
}

} // namespace owlet::platform::pico
