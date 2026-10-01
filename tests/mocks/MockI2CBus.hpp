#pragma once

#include "owlet/hal/II2CBus.hpp"

#include <gmock/gmock.h>

namespace owlet::test
{

// gmock fake for II2CBus. Used by host-side driver tests to assert the exact
// sequence of I2C transactions a driver emits.
class MockI2CBus : public ::owlet::hal::II2CBus
{
public:
    MOCK_METHOD(bool, writeRegister,
                (uint8_t devAddr, uint8_t reg, const uint8_t* data, std::size_t len),
                (override));
    MOCK_METHOD(bool, readRegister,
                (uint8_t devAddr, uint8_t reg, uint8_t* out, std::size_t len),
                (override));
    MOCK_METHOD(bool, probe, (uint8_t devAddr), (override));
};

} // namespace owlet::test
