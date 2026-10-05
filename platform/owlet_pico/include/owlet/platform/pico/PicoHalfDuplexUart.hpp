#pragma once

#include "owlet/hal/IHalfDuplexLink.hpp"

#include "hardware/uart.h"
#include "pico/time.h"

#include <cstddef>
#include <cstdint>

namespace owlet::platform::pico
{

// IHalfDuplexLink on a hardware UART whose TX and RX pins are both wired to
// one line (a TMC2209's PDN/UART pin).
//
// The TX pin only drives the line while a request is going out. The rest of
// the time it is a plain input, so the device can answer without two outputs
// fighting, and no series resistor is needed. Everything sent comes straight
// back on RX; that echo is read and dropped, and its absence means TX and RX
// are not really joined.
class PicoHalfDuplexUart final : public ::owlet::hal::IHalfDuplexLink
{
public:
    static constexpr unsigned kDefaultBaud = 115200;
    // Longest wait for any one byte. A device that will answer starts within
    // a few bit times, so this is generous and still bounds a failed
    // exchange at about 2 ms.
    static constexpr uint32_t kByteTimeoutUs = 2000;
    // Silence kept on the line after every exchange. A TMC2209 ignores a
    // request that follows the previous one too closely (found on hardware:
    // back-to-back requests made every second one fail).
    static constexpr uint32_t kInterFrameGapUs = 1000;

    PicoHalfDuplexUart(uart_inst_t* uart,
                       unsigned txPin,
                       unsigned rxPin,
                       unsigned baud = kDefaultBaud);

    PicoHalfDuplexUart(const PicoHalfDuplexUart&) = delete;
    PicoHalfDuplexUart& operator=(const PicoHalfDuplexUart&) = delete;

    // False if a pin number is out of range.
    [[nodiscard]] bool init();

    // Call from the control loop only; it blocks for the exchange (about
    // 1 ms for a TMC2209 register read at 115200 baud).
    [[nodiscard]] bool
    exchange(const uint8_t* tx, std::size_t txLen, uint8_t* rx, std::size_t rxLen) override;

private:
    [[nodiscard]] bool
    transfer(const uint8_t* tx, std::size_t txLen, uint8_t* rx, std::size_t rxLen);
    [[nodiscard]] bool readByte(uint8_t& out);
    void releaseLine();

    uart_inst_t* const m_uart;
    const unsigned m_txPin;
    const unsigned m_rxPin;
    const unsigned m_baud;
    absolute_time_t m_quietUntil{nil_time};
    bool m_ready{false};
};

} // namespace owlet::platform::pico
