#pragma once

#include "owlet/hal/IAxisCalibration.hpp"

namespace owlet::core
{

// Calibration of both axes, as persisted.
struct CalibrationRecord
{
    ::owlet::hal::AxisCalibration pan;
    ::owlet::hal::AxisCalibration tilt;
};

// Where calibration is kept between power cycles (NVS on ESP32, a flash
// sector on the Pico, a file in the simulator, an in-memory fake in tests).
class ICalibrationStore
{
public:
    virtual ~ICalibrationStore() = default;

    // False if nothing valid is stored.
    [[nodiscard]] virtual bool load(CalibrationRecord& out) = 0;
    [[nodiscard]] virtual bool save(const CalibrationRecord& record) = 0;
    [[nodiscard]] virtual bool erase() = 0;
};

} // namespace owlet::core
