#pragma once

#include "owlet/core/ICalibrationStore.hpp"

namespace owlet::platform::esp32
{

/// ICalibrationStore in NVS (namespace "owlet", key "cal"). The byte format
/// is core::CalibrationCodec (versioned + CRC), so a corrupt or older record
/// simply fails to load and the device falls back to factory values.
class Esp32NvsCalibrationStore final : public ::owlet::core::ICalibrationStore
{
public:
    /// Initialise the NVS partition (erasing it if its layout is from another
    /// IDF version). Call once before load/save.
    [[nodiscard]] bool init();

    [[nodiscard]] bool load(::owlet::core::CalibrationRecord& out) override;
    [[nodiscard]] bool save(const ::owlet::core::CalibrationRecord& record) override;
    [[nodiscard]] bool erase() override;
};

} // namespace owlet::platform::esp32
