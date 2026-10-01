#include "owlet/platform/esp32/Esp32NvsCalibrationStore.hpp"

#include "owlet/core/CalibrationCodec.hpp"

#include "nvs.h"
#include "nvs_flash.h"

namespace owlet::platform::esp32
{

namespace
{

constexpr const char* kNamespace = "owlet";
constexpr const char* kKey       = "cal";

// RAII for an NVS handle.
class NvsHandle
{
public:
    explicit NvsHandle(nvs_open_mode_t mode) : m_ok(nvs_open(kNamespace, mode, &m_handle) == ESP_OK) {}
    ~NvsHandle()
    {
        if (m_ok)
        {
            nvs_close(m_handle);
        }
    }
    NvsHandle(const NvsHandle&)            = delete;
    NvsHandle& operator=(const NvsHandle&) = delete;

    [[nodiscard]] bool ok() const { return m_ok; }
    nvs_handle_t get() const { return m_handle; }

private:
    nvs_handle_t m_handle{0};
    bool         m_ok;
};

} // namespace

bool Esp32NvsCalibrationStore::init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        if (nvs_flash_erase() != ESP_OK)
        {
            return false;
        }
        err = nvs_flash_init();
    }
    return err == ESP_OK;
}

bool Esp32NvsCalibrationStore::load(::owlet::core::CalibrationRecord& out)
{
    NvsHandle h(NVS_READONLY);
    if (!h.ok())
    {
        return false;  // namespace doesn't exist yet: nothing saved
    }
    ::owlet::core::CalibrationCodec::Bytes bytes{};
    size_t                                 len = bytes.size();
    if (nvs_get_blob(h.get(), kKey, bytes.data(), &len) != ESP_OK)
    {
        return false;
    }
    return ::owlet::core::CalibrationCodec::decode(bytes.data(), len, out);
}

bool Esp32NvsCalibrationStore::save(const ::owlet::core::CalibrationRecord& record)
{
    NvsHandle h(NVS_READWRITE);
    if (!h.ok())
    {
        return false;
    }
    const auto bytes = ::owlet::core::CalibrationCodec::encode(record);
    return nvs_set_blob(h.get(), kKey, bytes.data(), bytes.size()) == ESP_OK &&
           nvs_commit(h.get()) == ESP_OK;
}

bool Esp32NvsCalibrationStore::erase()
{
    NvsHandle h(NVS_READWRITE);
    if (!h.ok())
    {
        return false;
    }
    const esp_err_t err = nvs_erase_key(h.get(), kKey);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        return false;
    }
    return nvs_commit(h.get()) == ESP_OK;
}

} // namespace owlet::platform::esp32
