#pragma once

#include <cstddef>
#include <cstdint>

namespace owlet::hal
{

/// A request/reply link on one shared wire (a single-wire UART, RS-485, ...).
/// The master sends a request and the device answers on the same wire, so the
/// two never talk at once.
class IHalfDuplexLink
{
public:
    virtual ~IHalfDuplexLink() = default;

    /// Send `txLen` bytes, then receive exactly `rxLen` reply bytes into `rx`
    /// (`rxLen` may be 0 when no reply is expected). Blocks for the length of
    /// the exchange, which the implementation keeps short and bounded.
    //
    /// False if the reply did not arrive in full. A request with no reply
    /// cannot be confirmed at this level and reports true.
    [[nodiscard]] virtual bool
    exchange(const uint8_t* tx, std::size_t txLen, uint8_t* rx, std::size_t rxLen) = 0;
};

} // namespace owlet::hal
