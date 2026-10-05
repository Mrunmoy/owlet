#include "owlet/platform/pico/PicoUsbSerial.hpp"

#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"

#include <algorithm>
#include <climits>

namespace owlet::platform::pico
{

bool PicoUsbSerial::init()
{
    if (!stdio_usb_init())
    {
        return false;
    }
    stdio_set_translate_crlf(&stdio_usb, false);
    return true;
}

std::size_t PicoUsbSerial::read(uint8_t* out, std::size_t maxLen)
{
    if (out == nullptr || maxLen == 0)
    {
        return 0;
    }
    const int want = static_cast<int>(std::min<std::size_t>(maxLen, INT_MAX));
    const int n = stdio_usb.in_chars(reinterpret_cast<char*>(out), want);
    return (n > 0) ? static_cast<std::size_t>(n) : 0; // negative = nothing pending
}

bool PicoUsbSerial::write(const char* data, std::size_t len)
{
    if (data == nullptr || len > static_cast<std::size_t>(INT_MAX) || !stdio_usb_connected())
    {
        return false;
    }
    if (len != 0)
    {
        stdio_usb.out_chars(data, static_cast<int>(len));
    }
    return true;
}

} // namespace owlet::platform::pico
