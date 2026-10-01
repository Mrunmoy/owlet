#pragma once

#include "owlet/hal/IDelayProvider.hpp"
#include "owlet/hal/II2CBus.hpp"
#include "owlet/hal/IServoChannel.hpp"
#include "owlet/pca9685/Registers.hpp"

#include <cstdint>

namespace owlet
{

// Driver for the NXP PCA9685 16-channel 12-bit PWM controller. Platform-
// agnostic; wires itself to an II2CBus + IDelayProvider at construction.
//
// The typical lifecycle is:
//
//     Esp32I2CBus bus(...);
//     Esp32Delay  delay;
//     PCA9685 pwm(bus, delay);             // defaults: 0x40, 50 Hz
//     if (!pwm.init()) { /* probe or bus error */ }
//     pwm.setChannelPulseUs(0, 1500);      // neutral servo position
//
// Channels are also exposable as IServoChannel via `channel(i)`, which
// returns a lightweight proxy suitable for injection into owlet::Servo.
class PCA9685
{
public:
    struct Config
    {
        uint8_t  address         = ::owlet::pca9685::kDefaultAddress;
        uint32_t frequencyHz     = 50U;      // 50 Hz is the canonical servo rate
        bool     totemPoleOutput = true;     // OUTDRV=1 (datasheet default)
        bool     invertOutput    = false;    // MODE2.INVRT=0
    };

    // The two-arg form applies default Config (0x40, 50 Hz, totem-pole, non-inverted).
    // The three-arg form lets the caller override the config explicitly.
    // Two overloads instead of a default argument because gcc 11.x rejects
    // `Config cfg = {}` as a default for a type nested in the enclosing class.
    PCA9685(::owlet::hal::II2CBus&         bus,
            ::owlet::hal::IDelayProvider&  delay);
    PCA9685(::owlet::hal::II2CBus&         bus,
            ::owlet::hal::IDelayProvider&  delay,
            Config                         cfg);

    PCA9685(const PCA9685&)            = delete;
    PCA9685& operator=(const PCA9685&) = delete;

    // Probe the chip, sleep → program PRE_SCALE → wake → RESTART, configure MODE2.
    // Must succeed before any channel operation. Returns false on NACK / bus error.
    [[nodiscard]] bool init();

    // Reconfigure PWM frequency. Temporarily sleeps the oscillator (required by
    // datasheet footnote on PRE_SCALE), writes PRE_SCALE, wakes, waits 500 µs,
    // issues RESTART. Also updates the internally cached period so subsequent
    // `setChannelPulseUs` calls compute the correct off-count.
    [[nodiscard]] bool setFrequency(uint32_t hz);

    // Program a channel's active pulse width in microseconds. Emits one 4-byte
    // burst write to LEDn_ON_L..LEDn_OFF_H (auto-increment was enabled in init).
    [[nodiscard]] bool setChannelPulseUs(uint8_t channel, uint16_t pulseUs);

    // Direct 12-bit ON/OFF counter programming. Useful for LED dimming where
    // the caller prefers to work in counts instead of microseconds.
    [[nodiscard]] bool setChannelRaw(uint8_t channel, uint16_t onCount, uint16_t offCount);

    // Stop pulsing a channel (full-OFF bit in LEDn_OFF_H). Servo goes limp.
    [[nodiscard]] bool setChannelIdle(uint8_t channel);

    // One-shot "all channels off" via ALL_LED_OFF_H broadcast.
    [[nodiscard]] bool setAllIdle();

    // Manual sleep/wake. Usually not needed — init() and setFrequency() handle
    // them internally.
    [[nodiscard]] bool sleep();
    [[nodiscard]] bool wake();

    // Effective frequency actually programmed (PRE_SCALE is quantised, so the
    // realised rate may differ slightly from the requested Hz).
    [[nodiscard]] uint32_t frequencyHz() const { return m_actualFrequencyHz; }

    // Proxy exposing one channel as an IServoChannel for dependency injection
    // into logical-servo wrappers (owlet::Servo).
    class Channel final : public ::owlet::hal::IServoChannel
    {
    public:
        Channel(PCA9685& driver, uint8_t index) : m_driver(driver), m_index(index) {}

        [[nodiscard]] bool setPulseUs(uint16_t pulseUs) override
        {
            return m_driver.setChannelPulseUs(m_index, pulseUs);
        }
        [[nodiscard]] bool setIdle() override
        {
            return m_driver.setChannelIdle(m_index);
        }

    private:
        PCA9685& m_driver;
        uint8_t  m_index;
    };

    [[nodiscard]] Channel channel(uint8_t index) { return Channel(*this, index); }

private:
    static constexpr uint8_t kMode1AwakeFlags =
        ::owlet::pca9685::kMode1AutoInc | ::owlet::pca9685::kMode1AllCall;
    static constexpr uint8_t kMode1SleepFlags =
        ::owlet::pca9685::kMode1Sleep | kMode1AwakeFlags;
    static constexpr uint8_t kMode1RestartFlags =
        ::owlet::pca9685::kMode1Restart | kMode1AwakeFlags;
    static constexpr uint32_t kOscStabilizationUs = 500U;

    [[nodiscard]] bool writeMode1Raw(uint8_t value);
    [[nodiscard]] bool writeMode2FromConfig();
    [[nodiscard]] bool sleepAndWriteByte(uint8_t reg, uint8_t value);
    [[nodiscard]] bool writeLedBlock(uint8_t channel, uint16_t onCount, uint16_t offCount);
    [[nodiscard]] static bool validChannel(uint8_t channel) { return channel < 16; }

    ::owlet::hal::II2CBus&         m_bus;
    ::owlet::hal::IDelayProvider&  m_delay;
    Config                         m_config;
    uint32_t                       m_actualFrequencyHz{0};
    uint32_t                       m_pwmPeriodUs{0};
};

} // namespace owlet
