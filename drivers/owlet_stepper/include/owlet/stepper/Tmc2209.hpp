#pragma once

#include "owlet/hal/IHalfDuplexLink.hpp"

#include <cstddef>
#include <cstdint>

namespace owlet
{

// A TMC2209 over its single-wire UART: enough of the register set to take
// the current and microstep settings away from the potentiometer and the
// MS1/MS2 pins, and to read back load, stall and fault status.
//
// Every exchange blocks for about a millisecond at 115200 baud, so callers
// that share a control loop with motion should use Tmc2209Monitor, which
// spaces them out.
class Tmc2209
{
public:
    enum Register : uint8_t
    {
        kGconf = 0x00,
        kGstat = 0x01,
        kIfcnt = 0x02, // counts accepted writes: how a write is confirmed
        kIoin = 0x06,  // chip version in the top byte
        kIholdIrun = 0x10,
        kTcoolthrs = 0x14,
        kSgthrs = 0x40,
        kSgResult = 0x41,
        kChopconf = 0x6C,
        kDrvStatus = 0x6F,
    };

    struct Config
    {
        // The chip's address: MS1 is bit 0 and MS2 bit 1, as wired.
        uint8_t address{0};
        uint16_t runCurrentMa{450};  // RMS, while moving
        uint16_t holdCurrentMa{250}; // RMS, at standstill
        uint16_t microsteps{16};     // 1..256, a power of two
        uint16_t senseMilliOhm{110}; // sense resistors on the module
        // StallGuard threshold: the driver reports a stall when its load
        // value falls to twice this or below. 0 never reports one.
        uint8_t stallThreshold{0};
    };

    struct Status
    {
        bool overTempWarning{false};
        bool overTemp{false};
        bool shortA{false};
        bool shortB{false};
        bool openA{false}; // only meaningful while the motor is moving
        bool openB{false};
        bool standstill{false};
        uint8_t currentScale{0}; // 0..31, the scale actually in use
    };

    struct CurrentSetting
    {
        uint8_t scale;     // IRUN / IHOLD value, 0..31
        bool vsense;       // the high-sensitivity sense range
        uint16_t actualMa; // what that scale really gives, RMS
    };

    static constexpr uint8_t kVersion = 0x21;
    static constexpr uint16_t kDefaultSenseMilliOhm = 110;

    Tmc2209(::owlet::hal::IHalfDuplexLink& link, const Config& config);

    Tmc2209(const Tmc2209&) = delete;
    Tmc2209& operator=(const Tmc2209&) = delete;

    // ── Setup ───────────────────────────────────────────────────────────────

    // True if a TMC2209 answers at the configured address.
    [[nodiscard]] bool probe();

    // Write the configuration: current from the registers instead of the
    // potentiometer, microsteps from the register instead of MS1/MS2,
    // StealthChop on (StallGuard needs it), stall threshold. True only if the
    // chip counted every write.
    [[nodiscard]] bool configure();

    // ── Status ──────────────────────────────────────────────────────────────

    [[nodiscard]] bool readStatus(Status& out);
    // StallGuard load value, 0..510: high when the motor runs free, falling
    // toward 0 as it is loaded. Only meaningful while moving.
    [[nodiscard]] bool readLoad(uint16_t& out);
    // True in `wasReset` if the chip has restarted since configure() (its
    // supply dropped), in which case it is back on its defaults.
    [[nodiscard]] bool readResetFlag(bool& wasReset);

    [[nodiscard]] uint16_t runCurrentMa() const
    {
        return m_run.actualMa;
    }
    [[nodiscard]] uint8_t stallThreshold() const
    {
        return m_cfg.stallThreshold;
    }

    // ── Registers ───────────────────────────────────────────────────────────

    // `value` is left untouched unless a reply with a good CRC arrived.
    [[nodiscard]] bool readRegister(uint8_t reg, uint32_t& value);
    // Sent, not confirmed: the chip does not acknowledge a write. configure()
    // confirms a batch through the write counter.
    [[nodiscard]] bool writeRegister(uint8_t reg, uint32_t value);

    // ── Pure helpers (also used by tests and fakes) ─────────────────────────

    [[nodiscard]] static uint8_t crc8(const uint8_t* data, std::size_t len);
    [[nodiscard]] static CurrentSetting currentFor(uint16_t milliAmps, uint16_t senseMilliOhm);
    // CHOPCONF.MRES for a microstep count; anything else means 16.
    [[nodiscard]] static uint8_t microstepCode(uint16_t microsteps);

private:
    ::owlet::hal::IHalfDuplexLink& m_link;
    Config m_cfg;
    CurrentSetting m_run;
};

} // namespace owlet
