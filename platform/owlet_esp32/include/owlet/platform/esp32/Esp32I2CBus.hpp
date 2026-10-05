#pragma once

#include "owlet/hal/II2CBus.hpp"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace owlet::platform::esp32
{

/// ESP-IDF 5.x I²C master-driver implementation of II2CBus.
//
/// The new ESP-IDF I²C API is device-centric: each slave gets its own
/// `i2c_master_dev_handle_t`. We keep a small fixed-size cache indexed by
/// 7-bit address; the first transaction to a given address adds a device
/// handle lazily, and subsequent transactions reuse it.
class Esp32I2CBus final : public ::owlet::hal::II2CBus
{
public:
    struct Config
    {
        i2c_port_t port              = I2C_NUM_0;
        gpio_num_t sdaPin            = GPIO_NUM_1;   ///< Waveshare ESP32-S3 Zero: GP1
        gpio_num_t sclPin            = GPIO_NUM_2;   ///< Waveshare ESP32-S3 Zero: GP2
        uint32_t   clockHz           = 100'000U;     ///< standard-mode I²C
        bool       enableInternalPullups = true;
    };

    Esp32I2CBus();
    ~Esp32I2CBus();

    Esp32I2CBus(const Esp32I2CBus&)            = delete;
    Esp32I2CBus& operator=(const Esp32I2CBus&) = delete;

    /// Initialise the master bus with a caller-supplied config.
    [[nodiscard]] bool init(Config cfg);

    /// Convenience: initialise with the default Config (GP1/GP2, 100 kHz).
    [[nodiscard]] bool init();

    // ── II2CBus ─────────────────────────────────────────────────────────────
    [[nodiscard]] bool writeRegister(uint8_t devAddr, uint8_t reg,
                                     const uint8_t* data, std::size_t len) override;
    [[nodiscard]] bool readRegister(uint8_t devAddr, uint8_t reg,
                                    uint8_t* out, std::size_t len) override;
    [[nodiscard]] bool probe(uint8_t devAddr) override;

private:
    static constexpr std::size_t kMaxDevices   = 8;
    /// One register byte + N payload bytes. 16 is enough for LED-block
    /// bursts (4 bytes) with slack for future multi-channel batch writes.
    static constexpr std::size_t kMaxTxBytes   = 17;
    static constexpr uint32_t    kProbeTimeoutMs = 100U;
    static constexpr int         kTransactionTimeoutMs = -1; ///< wait forever

    struct DeviceSlot
    {
        uint8_t                 addr{0};
        i2c_master_dev_handle_t handle{nullptr};
    };

    [[nodiscard]] i2c_master_dev_handle_t lookupOrAdd(uint8_t addr);

    i2c_master_bus_handle_t         m_bus{nullptr};
    uint32_t                        m_clockHz{0};
    std::array<DeviceSlot, kMaxDevices> m_devices{};
    std::size_t                     m_deviceCount{0};
};

} // namespace owlet::platform::esp32
