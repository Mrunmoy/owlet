#include "owlet/sim/PCA9685Simulator.hpp"

namespace owlet::sim
{

namespace regs = ::owlet::pca9685;

PCA9685Simulator::PCA9685Simulator(uint8_t addr) : m_addr(addr)
{
    reset();
}

void PCA9685Simulator::reset()
{
    m_regs.fill(0x00);
    m_regs[regs::kRegMode1]        = regs::kMode1Default;
    m_regs[regs::kRegMode2]        = regs::kMode2Default;
    m_regs[regs::kRegSubAddr1]     = regs::kSubAddr1Default;
    m_regs[regs::kRegSubAddr2]     = regs::kSubAddr2Default;
    m_regs[regs::kRegSubAddr3]     = regs::kSubAddr3Default;
    m_regs[regs::kRegAllCallAddr]  = regs::kAllCallAdrDefault;
    m_regs[regs::kRegPreScale]     = regs::kPreScaleDefault;
    m_blockedPrescale = 0;
}

bool PCA9685Simulator::isReserved(uint8_t reg)
{
    // Range 0x46..0xF9 is "reserved for future use" and is not acknowledged.
    return reg > regs::kRegLed15OffH && reg < regs::kRegAllLedOnL;
}

uint8_t PCA9685Simulator::advanceAi(uint8_t reg)
{
    // Past the LED bank (reg 0x45) auto-increment wraps to MODE1.
    // From the ALL_LED block / PRE_SCALE (0xFA..0xFE) it also wraps to MODE1.
    // TestMode (0xFF) is reserved — we never advance into it.
    if (reg == regs::kRegLed15OffH || reg == regs::kRegPreScale)
    {
        return regs::kRegMode1;
    }
    return static_cast<uint8_t>(reg + 1U);
}

bool PCA9685Simulator::writeOne(uint8_t reg, uint8_t value)
{
    if (isReserved(reg))
    {
        return false;
    }

    // PRE_SCALE is write-blocked while awake (Table 4 footnote [1]).
    if (reg == regs::kRegPreScale && !isSleeping())
    {
        ++m_blockedPrescale;
        return true;  // bus ACKs; write is silently ignored
    }

    // ALL_LED_* bytes mirror into every LEDn_* register at the same offset.
    if (reg >= regs::kRegAllLedOnL && reg <= regs::kRegAllLedOffH)
    {
        const uint8_t offset = static_cast<uint8_t>(reg - regs::kRegAllLedOnL);
        for (uint8_t ch = 0; ch < 16; ++ch)
        {
            m_regs[regs::ledOnLReg(ch) + offset] = value;
        }
        m_regs[reg] = value;
        return true;
    }

    // MODE1.RESTART self-clears on write-1 (§7.3.1.1). Write-0 is a no-op.
    if (reg == regs::kRegMode1)
    {
        uint8_t incoming = value;
        if ((incoming & regs::kMode1Restart) != 0U)
        {
            incoming &= static_cast<uint8_t>(~regs::kMode1Restart);
        }
        m_regs[reg] = incoming;
        return true;
    }

    m_regs[reg] = value;
    return true;
}

bool PCA9685Simulator::onWrite(uint8_t reg, const uint8_t* data, std::size_t len)
{
    if (len == 0U) return true;
    if (data == nullptr) return false;

    uint8_t pointer = reg;
    const bool ai = autoIncrement();

    for (std::size_t i = 0; i < len; ++i)
    {
        if (!writeOne(pointer, data[i]))
        {
            return false;
        }
        if (ai && (i + 1U < len))
        {
            pointer = advanceAi(pointer);
        }
        // When AI=0, pointer stays the same for every subsequent byte in a
        // burst: successive writes hit the same register, and the last one wins.
    }
    return true;
}

bool PCA9685Simulator::onRead(uint8_t reg, uint8_t* out, std::size_t len)
{
    if (len == 0U) return true;
    if (out == nullptr) return false;
    if (isReserved(reg)) return false;

    uint8_t pointer = reg;
    const bool ai = autoIncrement();

    for (std::size_t i = 0; i < len; ++i)
    {
        if (isReserved(pointer)) return false;
        out[i] = m_regs[pointer];
        if (ai && (i + 1U < len))
        {
            pointer = advanceAi(pointer);
        }
    }
    return true;
}

uint16_t PCA9685Simulator::onCount(uint8_t channel) const
{
    const uint8_t base = regs::ledOnLReg(channel);
    const uint16_t high = static_cast<uint16_t>(m_regs[base + 1] & 0x0F);
    const uint16_t low  = static_cast<uint16_t>(m_regs[base + 0]);
    return static_cast<uint16_t>((high << 8) | low);
}

uint16_t PCA9685Simulator::offCount(uint8_t channel) const
{
    const uint8_t base = regs::ledOnLReg(channel) + 2U;
    const uint16_t high = static_cast<uint16_t>(m_regs[base + 1] & 0x0F);
    const uint16_t low  = static_cast<uint16_t>(m_regs[base + 0]);
    return static_cast<uint16_t>((high << 8) | low);
}

bool PCA9685Simulator::fullOn(uint8_t channel) const
{
    const uint8_t base = regs::ledOnLReg(channel);
    return (m_regs[base + 1] & regs::kLedFullOn) != 0U;
}

bool PCA9685Simulator::fullOff(uint8_t channel) const
{
    const uint8_t base = regs::ledOnLReg(channel);
    return (m_regs[base + 3] & regs::kLedFullOff) != 0U;
}

float PCA9685Simulator::frequencyHz() const
{
    return regs::frequencyForPrescale(m_regs[regs::kRegPreScale]);
}

} // namespace owlet::sim
