#pragma once

#include "owlet/hal/IDigitalOutput.hpp"

#include <cstdint>

// TMC2209 in standalone (step/dir) mode: the microstep resolution is set by
// two pins, and that is all there is to configure. Current is set by the
// module's potentiometer; the chip interpolates every step to 1/256.
namespace owlet::tmc2209
{

enum class Microsteps : uint8_t
{
    k8,
    k16,
    k32,
    k64,
};

[[nodiscard]] constexpr uint32_t microstepCount(Microsteps mode)
{
    switch (mode)
    {
    case Microsteps::k16:
        return 16;
    case Microsteps::k32:
        return 32;
    case Microsteps::k64:
        return 64;
    case Microsteps::k8:
        break;
    }
    return 8; // also what the chip does with both pins low
}

// MS2 MS1: 00 = 1/8, 01 = 1/32, 10 = 1/64, 11 = 1/16 (datasheet, standalone).
inline void selectMicrosteps(::owlet::hal::IDigitalOutput& ms1,
                             ::owlet::hal::IDigitalOutput& ms2,
                             Microsteps mode)
{
    ms1.set(mode == Microsteps::k32 || mode == Microsteps::k16);
    ms2.set(mode == Microsteps::k64 || mode == Microsteps::k16);
}

// Microsteps per degree at the output of a reduction (belt, gears).
[[nodiscard]] constexpr float
stepsPerDegree(uint32_t fullStepsPerRev, Microsteps mode, float reduction)
{
    return static_cast<float>(fullStepsPerRev * microstepCount(mode)) * reduction / 360.0f;
}

} // namespace owlet::tmc2209
