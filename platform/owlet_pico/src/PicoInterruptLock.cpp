#include "owlet/platform/pico/PicoInterruptLock.hpp"

#include "hardware/sync.h"

namespace owlet::platform::pico
{

uint32_t PicoInterruptLock::lock()
{
    return save_and_disable_interrupts();
}

void PicoInterruptLock::unlock(uint32_t saved)
{
    restore_interrupts(saved);
}

} // namespace owlet::platform::pico
