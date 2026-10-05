#pragma once

#include <cstddef>
#include <cstdint>

namespace owlet::hal
{

/// Register-addressed I2C master abstraction. Implementations live in
/// platform/owlet_esp32/ (real hardware) and simulators/ (host tests).
class II2CBus
{
public:
    virtual ~II2CBus() = default;

    /// Transmit `len` bytes to `devAddr` preceded by the `reg` pointer byte.
    /// Returns false on NACK or bus error.
    [[nodiscard]] virtual bool writeRegister(uint8_t devAddr,
                                             uint8_t reg,
                                             const uint8_t* data,
                                             std::size_t len) = 0;

    /// Write `reg` as pointer, then repeated start, then read `len` bytes into `out`.
    [[nodiscard]] virtual bool readRegister(uint8_t devAddr,
                                            uint8_t reg,
                                            uint8_t* out,
                                            std::size_t len) = 0;

    /// Returns true if a slave ACKs at `devAddr`.
    [[nodiscard]] virtual bool probe(uint8_t devAddr) = 0;

    /// Convenience helpers for single-byte register access. Non-virtual on purpose
    /// so implementations inherit a consistent semantics for free.
    [[nodiscard]] bool writeByte(uint8_t devAddr, uint8_t reg, uint8_t val)
    {
        return writeRegister(devAddr, reg, &val, 1);
    }

    [[nodiscard]] bool readByte(uint8_t devAddr, uint8_t reg, uint8_t& out)
    {
        return readRegister(devAddr, reg, &out, 1);
    }
};

} // namespace owlet::hal
