#include "owlet/platform/pico/PicoHalfDuplexUart.hpp"

#include "hardware/gpio.h"

namespace owlet::platform::pico
{

PicoHalfDuplexUart::PicoHalfDuplexUart(uart_inst_t* uart,
                                       unsigned txPin,
                                       unsigned rxPin,
                                       unsigned baud)
    : m_uart(uart), m_txPin(txPin), m_rxPin(rxPin), m_baud(baud)
{
}

bool PicoHalfDuplexUart::init()
{
    if (m_uart == nullptr || m_txPin >= NUM_BANK0_GPIOS || m_rxPin >= NUM_BANK0_GPIOS ||
        m_txPin == m_rxPin || m_baud == 0)
    {
        return false;
    }
    uart_init(m_uart, m_baud); // 8N1, FIFOs on
    gpio_set_function(m_rxPin, GPIO_FUNC_UART);
    gpio_pull_up(m_rxPin); // the line idles high even when nothing drives it
    releaseLine();
    m_ready = true;
    return true;
}

void PicoHalfDuplexUart::releaseLine()
{
    gpio_set_dir(m_txPin, GPIO_IN);
    gpio_disable_pulls(m_txPin);
    gpio_set_function(m_txPin, GPIO_FUNC_SIO);
}

bool PicoHalfDuplexUart::readByte(uint8_t& out)
{
    if (!uart_is_readable_within_us(m_uart, kByteTimeoutUs))
    {
        return false;
    }
    out = static_cast<uint8_t>(uart_getc(m_uart));
    return true;
}

bool PicoHalfDuplexUart::exchange(const uint8_t* tx,
                                  std::size_t txLen,
                                  uint8_t* rx,
                                  std::size_t rxLen)
{
    if (!m_ready || tx == nullptr || txLen == 0 || (rx == nullptr && rxLen != 0))
    {
        return false;
    }
    // Only ever waits when exchanges come back to back (a configuration
    // burst); the periodic status reads are far further apart than the gap.
    busy_wait_until(m_quietUntil);
    const bool ok = transfer(tx, txLen, rx, rxLen);
    m_quietUntil = make_timeout_time_us(kInterFrameGapUs);
    return ok;
}

bool PicoHalfDuplexUart::transfer(const uint8_t* tx,
                                  std::size_t txLen,
                                  uint8_t* rx,
                                  std::size_t rxLen)
{
    // Anything already waiting is noise or the tail of a reply nobody read.
    while (uart_is_readable(m_uart))
    {
        (void)uart_getc(m_uart);
    }

    gpio_set_function(m_txPin, GPIO_FUNC_UART); // drive the line
    uart_write_blocking(m_uart, tx, txLen);
    uart_tx_wait_blocking(m_uart); // until the last stop bit is out
    releaseLine();                 // before the device starts to answer

    uint8_t echo = 0;
    for (std::size_t i = 0; i < txLen; ++i)
    {
        if (!readByte(echo))
        {
            return false; // TX and RX are not on the same wire
        }
    }
    for (std::size_t i = 0; i < rxLen; ++i)
    {
        if (!readByte(rx[i]))
        {
            return false;
        }
    }
    return true;
}

} // namespace owlet::platform::pico
