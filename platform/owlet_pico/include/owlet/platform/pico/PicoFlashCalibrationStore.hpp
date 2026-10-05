#pragma once

#include "owlet/core/ICalibrationStore.hpp"

#include <cstdint>

namespace owlet::platform::pico
{

/// ICalibrationStore in the last sector of the on-board flash. The bytes are
/// CalibrationCodec's (magic, version, CRC), so a blank or foreign sector
/// simply reads as "nothing saved".
//
/// Saving and erasing stop the CPU from running code out of flash for the
/// length of a sector erase (tens of milliseconds, interrupts off). Step
/// bursts already queued in the PIO still finish, but the control loop
/// pauses, so save calibration while the head is at rest.
class PicoFlashCalibrationStore final : public ::owlet::core::ICalibrationStore
{
public:
    [[nodiscard]] bool load(::owlet::core::CalibrationRecord& out) override;
    [[nodiscard]] bool save(const ::owlet::core::CalibrationRecord& record) override;
    [[nodiscard]] bool erase() override;
};

} // namespace owlet::platform::pico
