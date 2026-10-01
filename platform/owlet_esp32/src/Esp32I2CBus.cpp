#include "owlet/platform/esp32/Esp32I2CBus.hpp"

#include "esp_err.h"

namespace owlet::platform::esp32
{

Esp32I2CBus::Esp32I2CBus() = default;

Esp32I2CBus::~Esp32I2CBus()
{
    for (std::size_t i = 0; i < m_deviceCount; ++i)
    {
        if (m_devices[i].handle != nullptr)
        {
            (void) i2c_master_bus_rm_device(m_devices[i].handle);
            m_devices[i].handle = nullptr;
        }
    }
    if (m_bus != nullptr)
    {
        (void) i2c_del_master_bus(m_bus);
        m_bus = nullptr;
    }
}

bool Esp32I2CBus::init(Config cfg)
{
    if (m_bus != nullptr) return false;

    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port          = cfg.port;
    busCfg.sda_io_num        = cfg.sdaPin;
    busCfg.scl_io_num        = cfg.sclPin;
    busCfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt = 7;
    busCfg.flags.enable_internal_pullup = cfg.enableInternalPullups;

    const esp_err_t err = i2c_new_master_bus(&busCfg, &m_bus);
    if (err != ESP_OK)
    {
        m_bus = nullptr;
        return false;
    }
    m_clockHz = cfg.clockHz;
    return true;
}

bool Esp32I2CBus::init()
{
    return init(Config{});
}

i2c_master_dev_handle_t Esp32I2CBus::lookupOrAdd(uint8_t addr)
{
    for (std::size_t i = 0; i < m_deviceCount; ++i)
    {
        if (m_devices[i].addr == addr) return m_devices[i].handle;
    }
    if (m_deviceCount >= kMaxDevices) return nullptr;
    if (m_bus == nullptr) return nullptr;

    i2c_device_config_t devCfg = {};
    devCfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    devCfg.device_address  = addr;
    devCfg.scl_speed_hz    = m_clockHz;

    i2c_master_dev_handle_t handle = nullptr;
    const esp_err_t err = i2c_master_bus_add_device(m_bus, &devCfg, &handle);
    if (err != ESP_OK || handle == nullptr) return nullptr;

    m_devices[m_deviceCount].addr   = addr;
    m_devices[m_deviceCount].handle = handle;
    ++m_deviceCount;
    return handle;
}

bool Esp32I2CBus::writeRegister(uint8_t devAddr, uint8_t reg,
                                const uint8_t* data, std::size_t len)
{
    if (len > kMaxTxBytes - 1) return false;
    if (len > 0 && data == nullptr) return false;

    i2c_master_dev_handle_t dev = lookupOrAdd(devAddr);
    if (dev == nullptr) return false;

    std::array<uint8_t, kMaxTxBytes> buf{};
    buf[0] = reg;
    for (std::size_t i = 0; i < len; ++i)
    {
        buf[i + 1] = data[i];
    }

    const esp_err_t err = i2c_master_transmit(dev, buf.data(), len + 1, kTransactionTimeoutMs);
    return err == ESP_OK;
}

bool Esp32I2CBus::readRegister(uint8_t devAddr, uint8_t reg,
                               uint8_t* out, std::size_t len)
{
    if (len == 0) return true;
    if (out == nullptr) return false;

    i2c_master_dev_handle_t dev = lookupOrAdd(devAddr);
    if (dev == nullptr) return false;

    const esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, out, len,
                                                      kTransactionTimeoutMs);
    return err == ESP_OK;
}

bool Esp32I2CBus::probe(uint8_t devAddr)
{
    if (m_bus == nullptr) return false;
    const esp_err_t err = i2c_master_probe(m_bus, devAddr, kProbeTimeoutMs);
    return err == ESP_OK;
}

} // namespace owlet::platform::esp32
