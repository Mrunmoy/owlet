#pragma once

#include <cstddef>
#include <cstdint>

namespace owlet::sim
{

// Slave-side interface paired with VirtualI2CBus. A simulated peripheral
// (such as the PCA9685 simulator) implements this and registers itself with
// a VirtualI2CBus, which routes incoming master transactions here by 7-bit
// address.
class II2CDevice
{
public:
    virtual ~II2CDevice() = default;

    virtual uint8_t address() const = 0;

    // Transaction callbacks mirror II2CBus semantics (register pointer + length).
    [[nodiscard]] virtual bool onWrite(uint8_t reg, const uint8_t* data, std::size_t len) = 0;
    [[nodiscard]] virtual bool onRead(uint8_t reg, uint8_t* out, std::size_t len) = 0;
    [[nodiscard]] virtual bool onProbe() = 0;
};

} // namespace owlet::sim
