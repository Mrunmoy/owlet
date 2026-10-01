#pragma once

#include "owlet/pca9685/Registers.hpp"
#include "owlet/sim/II2CDevice.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::sim
{

// Register-level PCA9685 simulator. Models everything the driver is going to
// touch: control register (register pointer), MODE1/MODE2, SUBADRn, ALLCALLADR,
// the LED0..LED15 ON/OFF register bank, the ALL_LED_* mirror registers,
// PRE_SCALE, auto-increment behavior, and the SLEEP-required guard on
// PRE_SCALE writes. Reserved address window (0x46..0xF9) returns false so
// drivers that poke it will surface as failures in tests.
//
// Not modeled:
//   - Oscillator stabilization timing (500 µs after SLEEP=0 → RESTART=1).
//     That belongs to a timing harness, not a register model.
//   - EXTCLK transition latching (the sticky-bit semantics). We accept the
//     bit but do not re-derive the refresh rate from an external source.
//   - I²C clock stretching, arbitration, or electrical errors. VirtualI2CBus
//     treats transactions as atomic.
class PCA9685Simulator final : public II2CDevice
{
public:
    explicit PCA9685Simulator(uint8_t addr = ::owlet::pca9685::kDefaultAddress);

    // ── II2CDevice overrides ────────────────────────────────────────────────

    uint8_t address() const override { return m_addr; }

    [[nodiscard]] bool onWrite(uint8_t reg, const uint8_t* data, std::size_t len) override;
    [[nodiscard]] bool onRead(uint8_t reg, uint8_t* out, std::size_t len) override;
    [[nodiscard]] bool onProbe() override { return true; }

    // ── Test introspection ──────────────────────────────────────────────────

    uint8_t regValue(uint8_t reg) const { return m_regs[reg]; }

    bool autoIncrement() const
    {
        return (m_regs[::owlet::pca9685::kRegMode1] & ::owlet::pca9685::kMode1AutoInc) != 0;
    }
    bool isSleeping() const
    {
        return (m_regs[::owlet::pca9685::kRegMode1] & ::owlet::pca9685::kMode1Sleep) != 0;
    }

    // 12-bit ON / OFF counters for LED channel 0..15.
    uint16_t onCount(uint8_t channel) const;
    uint16_t offCount(uint8_t channel) const;

    // True if LEDn_OFF_H.bit4 (full-OFF) is set for the given channel.
    bool fullOff(uint8_t channel) const;
    // True if LEDn_ON_H.bit4 (full-ON) is set for the given channel.
    bool fullOn(uint8_t channel) const;

    // Refresh rate (Hz) implied by current PRE_SCALE.
    float frequencyHz() const;

    // Count of PRE_SCALE writes that were silently dropped because SLEEP=0
    // (datasheet requires SLEEP=1 before PRE_SCALE can be updated).
    std::size_t blockedPrescaleWrites() const { return m_blockedPrescale; }

    // Re-apply power-on defaults. Clears write counters.
    void reset();

private:
    static uint8_t advanceAi(uint8_t reg);
    static bool    isReserved(uint8_t reg);

    // Apply a single-byte write, observing the side effects peculiar to
    // certain registers (ALL_LED_*, MODE1.RESTART, PRE_SCALE SLEEP guard,
    // reserved NACK). Returns the wire-level ACK/NACK.
    [[nodiscard]] bool writeOne(uint8_t reg, uint8_t value);

    uint8_t m_addr;
    std::array<uint8_t, 256> m_regs{};
    std::size_t m_blockedPrescale{0};
};

} // namespace owlet::sim
