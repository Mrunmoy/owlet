#include "owlet/stepper/Tmc2209.hpp"

#include <algorithm>
#include <array>

namespace owlet
{

namespace
{

constexpr uint8_t kSync = 0x05;
constexpr uint8_t kMasterAddress = 0xFF;
constexpr uint8_t kWriteFlag = 0x80;

// GCONF
constexpr uint32_t kIScaleAnalog = 1U << 0; // current reference from the VREF pin (the pot)
constexpr uint32_t kEnSpreadCycle = 1U << 2;
constexpr uint32_t kPdnDisable = 1U << 6;     // PDN pin is a UART only
constexpr uint32_t kMstepRegSelect = 1U << 7; // microsteps from CHOPCONF, not MS1/MS2
// CHOPCONF
constexpr uint32_t kVsense = 1U << 17;
constexpr uint32_t kMresShift = 24;
constexpr uint32_t kMresMask = 0xFU << kMresShift;
constexpr uint32_t kIntpol = 1U << 28;
// GSTAT
constexpr uint32_t kGstatReset = 1U << 0;
constexpr uint32_t kGstatAll = 0x7;
// DRV_STATUS
constexpr uint32_t kOtpw = 1U << 0;
constexpr uint32_t kOt = 1U << 1;
constexpr uint32_t kS2ga = 1U << 2;
constexpr uint32_t kS2gb = 1U << 3;
constexpr uint32_t kS2vsa = 1U << 4;
constexpr uint32_t kS2vsb = 1U << 5;
constexpr uint32_t kOla = 1U << 6;
constexpr uint32_t kOlb = 1U << 7;
constexpr uint32_t kStst = 1U << 31;

constexpr uint32_t kHoldDelay = 4;             // IHOLDDELAY: ramp down to hold current gently
constexpr uint32_t kStallAtAnySpeed = 0xFFFFF; // TCOOLTHRS: StallGuard active at every speed
constexpr uint32_t kConfigWrites = 6;

// Full-scale sense voltages (mV) and the 20 mΩ the datasheet adds for the
// internal path.
constexpr uint32_t kVfsLowMv = 180;  // vsense = 1
constexpr uint32_t kVfsHighMv = 325; // vsense = 0
constexpr uint32_t kInternalMilliOhm = 20;

// Full-scale RMS current in mA: Vfs / (Rsense + 20 mΩ) / √2.
constexpr uint32_t fullScaleMa(uint32_t vfsMv, uint32_t senseMilliOhm)
{
    return vfsMv * 1'000'000U / ((senseMilliOhm + kInternalMilliOhm) * 1414U);
}

constexpr uint8_t scaleFor(uint32_t milliAmps, uint32_t fullScale)
{
    // (CS + 1) / 32 of full scale, rounded to nearest.
    const uint32_t steps = (32U * milliAmps + fullScale / 2U) / fullScale;
    return static_cast<uint8_t>(std::clamp<uint32_t>(steps, 1U, 32U) - 1U);
}

} // namespace

Tmc2209::Tmc2209(::owlet::hal::IHalfDuplexLink& link, const Config& config)
    : m_link(link), m_cfg(config), m_run(currentFor(config.runCurrentMa, config.senseMilliOhm))
{
}

// ── Setup ───────────────────────────────────────────────────────────────────

bool Tmc2209::probe()
{
    uint32_t ioin = 0;
    return readRegister(kIoin, ioin) && static_cast<uint8_t>(ioin >> 24) == kVersion;
}

bool Tmc2209::configure()
{
    uint32_t before = 0;
    uint32_t gconf = 0;
    uint32_t chop = 0;
    if (!readRegister(kIfcnt, before) || !readRegister(kGconf, gconf) ||
        !readRegister(kChopconf, chop))
    {
        return false;
    }
    gconf |= kPdnDisable | kMstepRegSelect;
    gconf &= ~(kIScaleAnalog | kEnSpreadCycle);

    chop &= ~(kMresMask | kVsense);
    chop |= static_cast<uint32_t>(microstepCode(m_cfg.microsteps)) << kMresShift;
    chop |= kIntpol;
    if (m_run.vsense)
    {
        chop |= kVsense;
    }

    // Hold current uses the run current's sense range, and never exceeds it.
    const uint32_t sense = (m_cfg.senseMilliOhm != 0) ? m_cfg.senseMilliOhm : kDefaultSenseMilliOhm;
    const uint32_t fullScale = fullScaleMa(m_run.vsense ? kVfsLowMv : kVfsHighMv, sense);
    const uint8_t hold = std::min(scaleFor(m_cfg.holdCurrentMa, fullScale), m_run.scale);
    const uint32_t current = static_cast<uint32_t>(hold) |
                             (static_cast<uint32_t>(m_run.scale) << 8) | (kHoldDelay << 16);

    if (!writeRegister(kGconf, gconf) || !writeRegister(kChopconf, chop) ||
        !writeRegister(kIholdIrun, current) || !writeRegister(kTcoolthrs, kStallAtAnySpeed) ||
        !writeRegister(kSgthrs, m_cfg.stallThreshold) || !writeRegister(kGstat, kGstatAll))
    {
        return false;
    }
    uint32_t after = 0;
    return readRegister(kIfcnt, after) && ((after - before) & 0xFFU) == kConfigWrites;
}

// ── Status ──────────────────────────────────────────────────────────────────

bool Tmc2209::readStatus(Status& out)
{
    uint32_t v = 0;
    if (!readRegister(kDrvStatus, v))
    {
        return false;
    }
    out.overTempWarning = (v & kOtpw) != 0;
    out.overTemp = (v & kOt) != 0;
    out.shortA = (v & (kS2ga | kS2vsa)) != 0;
    out.shortB = (v & (kS2gb | kS2vsb)) != 0;
    out.openA = (v & kOla) != 0;
    out.openB = (v & kOlb) != 0;
    out.standstill = (v & kStst) != 0;
    out.currentScale = static_cast<uint8_t>((v >> 16) & 0x1FU);
    return true;
}

bool Tmc2209::readLoad(uint16_t& out)
{
    uint32_t v = 0;
    if (!readRegister(kSgResult, v))
    {
        return false;
    }
    out = static_cast<uint16_t>(v & 0x3FFU);
    return true;
}

bool Tmc2209::readResetFlag(bool& wasReset)
{
    uint32_t v = 0;
    if (!readRegister(kGstat, v))
    {
        return false;
    }
    wasReset = (v & kGstatReset) != 0;
    return true;
}

// ── Registers ───────────────────────────────────────────────────────────────

bool Tmc2209::readRegister(uint8_t reg, uint32_t& value)
{
    std::array<uint8_t, 4> request{kSync, m_cfg.address, static_cast<uint8_t>(reg & 0x7FU), 0};
    request[3] = crc8(request.data(), 3);
    std::array<uint8_t, 8> reply{};
    if (!m_link.exchange(request.data(), request.size(), reply.data(), reply.size()))
    {
        return false;
    }
    if (reply[0] != kSync || reply[1] != kMasterAddress || reply[2] != request[2] ||
        crc8(reply.data(), 7) != reply[7])
    {
        return false;
    }
    value = (static_cast<uint32_t>(reply[3]) << 24) | (static_cast<uint32_t>(reply[4]) << 16) |
            (static_cast<uint32_t>(reply[5]) << 8) | reply[6];
    return true;
}

bool Tmc2209::writeRegister(uint8_t reg, uint32_t value)
{
    std::array<uint8_t, 8> request{kSync,
                                   m_cfg.address,
                                   static_cast<uint8_t>(reg | kWriteFlag),
                                   static_cast<uint8_t>(value >> 24),
                                   static_cast<uint8_t>(value >> 16),
                                   static_cast<uint8_t>(value >> 8),
                                   static_cast<uint8_t>(value),
                                   0};
    request[7] = crc8(request.data(), 7);
    return m_link.exchange(request.data(), request.size(), nullptr, 0);
}

// ── Pure helpers ────────────────────────────────────────────────────────────

uint8_t Tmc2209::crc8(const uint8_t* data, std::size_t len)
{
    // CRC-8, polynomial 0x07, each byte taken LSB first (datasheet §4.2).
    uint8_t crc = 0;
    for (std::size_t i = 0; i < len; ++i)
    {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            const bool feedback = (((crc >> 7) ^ (byte & 1U)) & 1U) != 0;
            crc = static_cast<uint8_t>(crc << 1);
            if (feedback)
            {
                crc = static_cast<uint8_t>(crc ^ 0x07U);
            }
            byte = static_cast<uint8_t>(byte >> 1);
        }
    }
    return crc;
}

Tmc2209::CurrentSetting Tmc2209::currentFor(uint16_t milliAmps, uint16_t senseMilliOhm)
{
    const uint32_t sense = (senseMilliOhm != 0) ? senseMilliOhm : kDefaultSenseMilliOhm;
    const uint32_t low = fullScaleMa(kVfsLowMv, sense);
    // The low range gives finer steps; use it whenever it reaches far enough.
    const bool vsense = milliAmps <= low;
    const uint32_t fullScale = vsense ? low : fullScaleMa(kVfsHighMv, sense);
    const uint8_t scale = scaleFor(milliAmps, fullScale);
    return CurrentSetting{
        scale,
        vsense,
        static_cast<uint16_t>((static_cast<uint32_t>(scale) + 1U) * fullScale / 32U)};
}

uint8_t Tmc2209::microstepCode(uint16_t microsteps)
{
    // MRES: 0 = 256 microsteps, halving up to 8 = full step.
    for (uint8_t code = 0; code <= 8; ++code)
    {
        if (microsteps == (256U >> code))
        {
            return code;
        }
    }
    return 4; // 16
}

} // namespace owlet
