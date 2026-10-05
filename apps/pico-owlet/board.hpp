#pragma once

// Everything that describes this particular head: which pin does what, the
// motors and belts, the driver settings, speeds and limits. To build a head
// that is wired or geared differently, change this file (or copy it); the
// wiring in main.cpp does not need to know.

#include "owlet/platform/pico/PicoDigitalInput.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"

#include "pico/stdlib.h"

#include <cstddef>
#include <cstdint>

namespace board
{

using ::owlet::StepperAxis;
using ::owlet::platform::pico::PicoDigitalInput;
using ::owlet::tmc2209::Microsteps;

// ── Board ───────────────────────────────────────────────────────────────────

constexpr unsigned kPanStepPin = 2;
constexpr unsigned kPanDirPin = 3;
constexpr unsigned kTiltStepPin = 7; // as built: the reverse of the pan pair
constexpr unsigned kTiltDirPin = 6;
constexpr unsigned kEnablePin = 8; // both drivers, active low
constexpr unsigned kMs1Pin = 12;   // both drivers
constexpr unsigned kMs2Pin = 13;   // both drivers
// Each driver's PDN/UART pin has its own serial line (TX and RX joined at the
// driver), so the two need no distinct addresses and MS1/MS2 stay shared.
constexpr unsigned kPanUartTxPin = 4; // UART1
constexpr unsigned kPanUartRxPin = 5;
constexpr unsigned kTiltUartTxPin = 0; // UART0
constexpr unsigned kTiltUartRxPin = 1;
constexpr unsigned kHeartbeatLedPin = PICO_DEFAULT_LED_PIN; // the LED on the board
constexpr unsigned kActivityLedPin = 22;                    // sinks the LED: low = lit

// ── Limit switches ──────────────────────────────────────────────────────────
// Two per axis, one at each end of travel. Set kLimitSwitchesFitted once
// they are wired (docs/wiring/pico-tmc2209); until then none of these pins is
// touched and the head behaves exactly as it did without them.
//
// Wire each switch normally-closed between its pin and ground. The pin's
// pull-up then reads "pressed" when the switch opens, and also when a wire
// breaks or a plug comes out, so a wiring fault stops the axis.

constexpr bool kLimitSwitchesFitted = false;
constexpr unsigned kPanMinSwitchPin = 10;
constexpr unsigned kPanMaxSwitchPin = 11;
constexpr unsigned kTiltMinSwitchPin = 14;
constexpr unsigned kTiltMaxSwitchPin = 15;
constexpr PicoDigitalInput::Polarity kSwitchPolarity = PicoDigitalInput::Polarity::ActiveHigh;
constexpr PicoDigitalInput::Pull kSwitchPull = PicoDigitalInput::Pull::Up;

// ── Mechanism ───────────────────────────────────────────────────────────────

constexpr uint32_t kFullStepsPerRev = 200; // 1.8° motors
constexpr Microsteps kMicrosteps = Microsteps::k16;
constexpr float kBeltReduction = 3.0f; // 20T motor pulley : 60T output

// ── Drivers over UART ───────────────────────────────────────────────────────
// When a TMC2209 answers on its UART the firmware takes the current and the
// microstep setting over from the potentiometer and the MS pins. When it
// does not (motor supply off, or no UART wired) the head runs standalone,
// exactly as before: the same 1/16 comes from the MS pins.

constexpr ::owlet::Tmc2209::Config makeDriver()
{
    ::owlet::Tmc2209::Config c;
    // MS1 and MS2 are both high for 1/16 in standalone mode, and those two
    // pins are also the UART address: 3.
    c.address = 3;
    // About what the potentiometers were giving when the head was tested
    // (0.23 A from the supply for both motors holding).
    c.runCurrentMa = 450;
    c.holdCurrentMa = 250;
    c.microsteps = static_cast<uint16_t>(::owlet::tmc2209::microstepCount(kMicrosteps));
    c.senseMilliOhm = 110;
    c.stallThreshold = 0; // stall reporting off until the load values are characterised
    return c;
}

constexpr ::owlet::Tmc2209::Config kDriver = makeDriver();

// Flip if an axis turns the wrong way. Pan + must be the camera's own right
// (the left of someone facing it) and tilt + up. Both confirmed by eye on the
// reference head.
constexpr bool kPanReversed = true;
constexpr bool kTiltReversed = true;

// About two thirds of what ran clean on the reference head: 360 °/s and
// 1920 °/s² on both axes at once, on 12 V, with no lost steps
// (docs/CALIBRATION.md). Clients read the limits with $CAPS and adapt.
constexpr float kMaxSpeedDegPerSec = 240.0f;
constexpr float kAccelDegPerSec2 = 1440.0f;

constexpr StepperAxis::Config makeAxis(bool reversed, float limitDeg, float travelDeg)
{
    StepperAxis::Config c;
    c.stepsPerDeg = ::owlet::tmc2209::stepsPerDegree(kFullStepsPerRev, kMicrosteps, kBeltReduction);
    c.reversed = reversed;
    c.maxSpeedDegPerSec = kMaxSpeedDegPerSec;
    c.accelDegPerSec2 = kAccelDegPerSec2;
    c.minDeg = -limitDeg;
    c.maxDeg = limitDeg;
    c.travelMinDeg = -travelDeg;
    c.travelMaxDeg = travelDeg;
    return c;
}

// Factory soft limits, and the widest a calibration may set them (cable
// wrap on pan, the cradle against the yoke on tilt). Pan ±90 and tilt ±60
// were swept on the reference head (docs/CALIBRATION.md).
constexpr StepperAxis::Config kPanAxis = makeAxis(kPanReversed, 90.0f, 170.0f);
constexpr StepperAxis::Config kTiltAxis = makeAxis(kTiltReversed, 60.0f, 90.0f);

// 500 Hz: each tick hands the PIO one 2 ms burst per axis.
constexpr uint32_t kLoopPeriodUs = 2000;

// Events waiting between two loop passes, and the listeners they go to.
constexpr std::size_t kEventQueueDepth = 16;
constexpr std::size_t kEventListeners = 2;

} // namespace board
