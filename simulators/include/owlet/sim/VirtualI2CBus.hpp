#pragma once

#include "owlet/hal/II2CBus.hpp"
#include "owlet/sim/II2CDevice.hpp"

#include <cstddef>
#include <vector>

namespace owlet::sim
{

// Host-side I2C master that dispatches each transaction to whichever attached
// simulated device matches the 7-bit address. Unknown addresses return false
// (NACK). Suitable only for host tests — never included on the target.
class VirtualI2CBus final : public ::owlet::hal::II2CBus
{
public:
    // Register a device so transactions to its address route to it.
    void attach(II2CDevice& device);

    // ── II2CBus ──
    [[nodiscard]] bool writeRegister(uint8_t devAddr, uint8_t reg,
                                     const uint8_t* data, std::size_t len) override;
    [[nodiscard]] bool readRegister(uint8_t devAddr, uint8_t reg,
                                    uint8_t* out, std::size_t len) override;
    [[nodiscard]] bool probe(uint8_t devAddr) override;

    // ── Test introspection ──
    std::size_t writeCount() const { return m_writeCount; }
    std::size_t readCount() const { return m_readCount; }
    std::size_t probeCount() const { return m_probeCount; }

private:
    II2CDevice* find(uint8_t addr) const;

    std::vector<II2CDevice*> m_devices;
    std::size_t m_writeCount{0};
    std::size_t m_readCount{0};
    std::size_t m_probeCount{0};
};

} // namespace owlet::sim
