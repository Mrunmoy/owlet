#include "owlet/platform/esp32/Esp32UsbSerial.hpp"

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace owlet::platform::esp32
{

bool Esp32UsbSerial::init()
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = kRxBufferBytes;
    cfg.tx_buffer_size = kTxBufferBytes;
    return usb_serial_jtag_driver_install(&cfg) == ESP_OK;
}

std::size_t Esp32UsbSerial::read(uint8_t* out, std::size_t maxLen)
{
    const int n = usb_serial_jtag_read_bytes(out, static_cast<uint32_t>(maxLen), 0);
    return n > 0 ? static_cast<std::size_t>(n) : 0;
}

bool Esp32UsbSerial::write(const char* data, std::size_t len)
{
    // With no host attached the TX ring never drains: skip the write so the
    // control loop stays on schedule.
    if (!usb_serial_jtag_is_connected())
    {
        return false;
    }
    // A connected but momentarily slow host gets a short grace period rather
    // than a silently dropped reply (each request is owed exactly one). The
    // watchdog runs on wall time, so this delay can't extend motion.
    const int n = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(kWriteTimeoutMs));
    return n == static_cast<int>(len);
}

} // namespace owlet::platform::esp32
