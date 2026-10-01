#pragma once

#include <cstdint>

// PCA9685 register map, bit flags, and power-on defaults. Derived from the
// NXP PCA9685 product data sheet, Rev. 4 — 16 April 2015, Tables 4–6 and §7.
//
// Defined once here so the driver (drivers/owlet_pca9685) and the simulator
// (simulators/ under owlet::sim) agree on register addresses and masks. No
// functions — constants only.

namespace owlet::pca9685
{

// ── Bus + oscillator ────────────────────────────────────────────────────────

inline constexpr uint8_t  kDefaultAddress  = 0x40;   // A5..A0 tied LOW
inline constexpr uint8_t  kSoftwareResetAddr = 0x03; // SWRST, write-only, R/W=0
inline constexpr uint32_t kInternalClockHz = 25'000'000U;
inline constexpr uint16_t kPwmCounterMax   = 4096;   // 12-bit counter 0..4095

// ── Register addresses (datasheet Table 4) ──────────────────────────────────

inline constexpr uint8_t kRegMode1         = 0x00;
inline constexpr uint8_t kRegMode2         = 0x01;
inline constexpr uint8_t kRegSubAddr1      = 0x02;
inline constexpr uint8_t kRegSubAddr2      = 0x03;
inline constexpr uint8_t kRegSubAddr3      = 0x04;
inline constexpr uint8_t kRegAllCallAddr   = 0x05;
inline constexpr uint8_t kRegLed0OnL       = 0x06;  // LEDn block: 4 regs per channel
inline constexpr uint8_t kRegLed15OffH     = 0x45;
inline constexpr uint8_t kRegAllLedOnL     = 0xFA;
inline constexpr uint8_t kRegAllLedOnH     = 0xFB;
inline constexpr uint8_t kRegAllLedOffL    = 0xFC;
inline constexpr uint8_t kRegAllLedOffH    = 0xFD;
inline constexpr uint8_t kRegPreScale      = 0xFE;
inline constexpr uint8_t kRegTestMode      = 0xFF;   // reserved

inline constexpr uint8_t kLedRegistersPerChannel = 4;  // ON_L, ON_H, OFF_L, OFF_H

// ── MODE1 bits (datasheet Table 5) ──────────────────────────────────────────

inline constexpr uint8_t kMode1Restart = 0x80;  // write 1 to clear
inline constexpr uint8_t kMode1ExtClk  = 0x40;  // external clock select (sticky)
inline constexpr uint8_t kMode1AutoInc = 0x20;  // register auto-increment
inline constexpr uint8_t kMode1Sleep   = 0x10;  // 1=low-power, oscillator off
inline constexpr uint8_t kMode1Sub1    = 0x08;
inline constexpr uint8_t kMode1Sub2    = 0x04;
inline constexpr uint8_t kMode1Sub3    = 0x02;
inline constexpr uint8_t kMode1AllCall = 0x01;

// ── MODE2 bits (datasheet Table 6) ──────────────────────────────────────────

inline constexpr uint8_t kMode2Invrt       = 0x10;
inline constexpr uint8_t kMode2Och         = 0x08;  // 0=change on STOP, 1=on ACK
inline constexpr uint8_t kMode2OutDrv      = 0x04;  // 0=open-drain, 1=totem pole
inline constexpr uint8_t kMode2OutNeMask   = 0x03;

// LEDn_ON_H[4] / LEDn_OFF_H[4]: full-ON / full-OFF bits. Full-OFF wins over Full-ON.
inline constexpr uint8_t kLedFullOn  = 0x10;
inline constexpr uint8_t kLedFullOff = 0x10;

// ── Power-on defaults (datasheet Tables 5, 6 and §7.1.2–7.1.3) ──────────────

inline constexpr uint8_t kMode1Default      = kMode1Sleep | kMode1AllCall; // 0x11
inline constexpr uint8_t kMode2Default      = kMode2OutDrv;                // 0x04
inline constexpr uint8_t kSubAddr1Default   = 0xE2;
inline constexpr uint8_t kSubAddr2Default   = 0xE4;
inline constexpr uint8_t kSubAddr3Default   = 0xE8;
inline constexpr uint8_t kAllCallAdrDefault = 0xE0;
inline constexpr uint8_t kPreScaleDefault   = 0x1E;   // ≈ 200 Hz at 25 MHz

// ── Helpers ─────────────────────────────────────────────────────────────────

// Register address of LEDn_ON_L for channel 0..15.
[[nodiscard]] constexpr uint8_t ledOnLReg(uint8_t channel)
{
    return static_cast<uint8_t>(kRegLed0OnL + channel * kLedRegistersPerChannel);
}

// Convert a requested frequency in Hz to the PRE_SCALE register value,
// using the datasheet formula with round-to-nearest.
//   prescale = round(fosc / (4096 · f)) − 1
// Clamped into the valid 0x03..0xFF range.
[[nodiscard]] constexpr uint8_t prescaleForFrequency(uint32_t freqHz)
{
    if (freqHz == 0U)
    {
        return 0xFF;
    }
    const uint32_t denom = static_cast<uint32_t>(kPwmCounterMax) * freqHz;
    const uint32_t rounded = (kInternalClockHz + denom / 2U) / denom;
    if (rounded == 0U)
    {
        return 0x03;
    }
    const uint32_t prescale = rounded - 1U;
    if (prescale < 0x03U) return 0x03;
    if (prescale > 0xFFU) return 0xFF;
    return static_cast<uint8_t>(prescale);
}

// Refresh rate (Hz) implied by a given PRE_SCALE value.
[[nodiscard]] constexpr float frequencyForPrescale(uint8_t prescale)
{
    return static_cast<float>(kInternalClockHz) /
           static_cast<float>(kPwmCounterMax * (prescale + 1U));
}

} // namespace owlet::pca9685
