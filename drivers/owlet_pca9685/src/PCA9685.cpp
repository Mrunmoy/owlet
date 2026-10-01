#include "owlet/pca9685/PCA9685.hpp"

namespace owlet
{

namespace regs = ::owlet::pca9685;

PCA9685::PCA9685(::owlet::hal::II2CBus&         bus,
                 ::owlet::hal::IDelayProvider&  delay)
    : PCA9685(bus, delay, Config{})
{
}

PCA9685::PCA9685(::owlet::hal::II2CBus&         bus,
                 ::owlet::hal::IDelayProvider&  delay,
                 Config                         cfg)
    : m_bus(bus), m_delay(delay), m_config(cfg)
{
}

bool PCA9685::init()
{
    if (!m_bus.probe(m_config.address))
    {
        return false;
    }
    if (!setFrequency(m_config.frequencyHz))
    {
        return false;
    }
    if (!writeMode2FromConfig())
    {
        return false;
    }
    return true;
}

bool PCA9685::setFrequency(uint32_t hz)
{
    if (hz == 0U)
    {
        return false;
    }

    const uint8_t prescale = regs::prescaleForFrequency(hz);

    // Datasheet: PRE_SCALE is write-blocked unless SLEEP=1. Sleep, set, wake.
    if (!writeMode1Raw(kMode1SleepFlags))
    {
        return false;
    }
    if (!m_bus.writeByte(m_config.address, regs::kRegPreScale, prescale))
    {
        return false;
    }
    if (!writeMode1Raw(kMode1AwakeFlags))
    {
        return false;
    }

    // Oscillator needs ≥500 µs to stabilize before PWM timings are guaranteed.
    m_delay.delayUs(kOscStabilizationUs);

    // RESTART self-clears; use it so any paused PWM state picks back up.
    if (!writeMode1Raw(kMode1RestartFlags))
    {
        return false;
    }

    m_actualFrequencyHz = static_cast<uint32_t>(regs::frequencyForPrescale(prescale));
    if (m_actualFrequencyHz == 0U)
    {
        return false;
    }
    m_pwmPeriodUs = 1'000'000U / m_actualFrequencyHz;
    return true;
}

bool PCA9685::setChannelPulseUs(uint8_t channel, uint16_t pulseUs)
{
    if (!validChannel(channel))
    {
        return false;
    }
    if (m_pwmPeriodUs == 0U)
    {
        return false;  // init() not called
    }
    if (pulseUs >= m_pwmPeriodUs)
    {
        return false;  // caller must use setAllIdle / setChannelIdle for full-off
    }

    const uint32_t off = (static_cast<uint32_t>(pulseUs) * regs::kPwmCounterMax)
                         / m_pwmPeriodUs;
    return writeLedBlock(channel, 0U, static_cast<uint16_t>(off));
}

bool PCA9685::setChannelRaw(uint8_t channel, uint16_t onCount, uint16_t offCount)
{
    if (!validChannel(channel))
    {
        return false;
    }
    if (onCount >= regs::kPwmCounterMax || offCount >= regs::kPwmCounterMax)
    {
        return false;
    }
    return writeLedBlock(channel, onCount, offCount);
}

bool PCA9685::setChannelIdle(uint8_t channel)
{
    if (!validChannel(channel))
    {
        return false;
    }
    const uint8_t buf[4] = {
        0x00, 0x00,
        0x00, regs::kLedFullOff,
    };
    return m_bus.writeRegister(m_config.address, regs::ledOnLReg(channel), buf, 4);
}

bool PCA9685::setAllIdle()
{
    return m_bus.writeByte(m_config.address,
                           regs::kRegAllLedOffH,
                           regs::kLedFullOff);
}

bool PCA9685::sleep()
{
    return writeMode1Raw(kMode1SleepFlags);
}

bool PCA9685::wake()
{
    if (!writeMode1Raw(kMode1AwakeFlags))
    {
        return false;
    }
    m_delay.delayUs(kOscStabilizationUs);
    return true;
}

// ── Private helpers ─────────────────────────────────────────────────────────

bool PCA9685::writeMode1Raw(uint8_t value)
{
    return m_bus.writeByte(m_config.address, regs::kRegMode1, value);
}

bool PCA9685::writeMode2FromConfig()
{
    uint8_t mode2 = 0;
    if (m_config.totemPoleOutput) mode2 |= regs::kMode2OutDrv;
    if (m_config.invertOutput)    mode2 |= regs::kMode2Invrt;
    return m_bus.writeByte(m_config.address, regs::kRegMode2, mode2);
}

bool PCA9685::sleepAndWriteByte(uint8_t reg, uint8_t value)
{
    if (!writeMode1Raw(kMode1SleepFlags)) return false;
    if (!m_bus.writeByte(m_config.address, reg, value)) return false;
    return writeMode1Raw(kMode1AwakeFlags);
}

bool PCA9685::writeLedBlock(uint8_t channel, uint16_t onCount, uint16_t offCount)
{
    const uint8_t buf[4] = {
        static_cast<uint8_t>(onCount & 0xFFU),
        static_cast<uint8_t>((onCount >> 8) & 0x0FU),
        static_cast<uint8_t>(offCount & 0xFFU),
        static_cast<uint8_t>((offCount >> 8) & 0x0FU),
    };
    return m_bus.writeRegister(m_config.address, regs::ledOnLReg(channel), buf, 4);
}

} // namespace owlet
