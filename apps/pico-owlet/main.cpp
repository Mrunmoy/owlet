// owlet firmware entry point for the Raspberry Pi Pico stepper head:
// composition root only.
//
// Builds the platform objects, injects them into the platform-agnostic core,
// and runs the service loop. All behaviour lives in firmware/common and
// drivers/ (host-tested); nothing here makes decisions. The core, the
// protocol and the PC client are the same ones the ESP32 servo head runs.
//
// Wiring: docs/wiring/pico-tmc2209/.

#include "owlet/core/ActivityIndicator.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/platform/pico/PicoClock.hpp"
#include "owlet/platform/pico/PicoDigitalOutput.hpp"
#include "owlet/platform/pico/PicoFlashCalibrationStore.hpp"
#include "owlet/platform/pico/PicoStepGenerator.hpp"
#include "owlet/platform/pico/PicoUsbSerial.hpp"
#include "owlet/stepper/SharedEnable.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"
#include "owlet/text/TextProtocol.hpp"

#include "hardware/pio.h"
#include "pico/stdlib.h"

#include <cstdint>

namespace
{

using ::owlet::SharedEnable;
using ::owlet::StepperAxis;
using ::owlet::platform::pico::PicoClock;
using ::owlet::platform::pico::PicoDigitalOutput;
using ::owlet::platform::pico::PicoFlashCalibrationStore;
using ::owlet::platform::pico::PicoStepGenerator;
using ::owlet::platform::pico::PicoStepProgram;
using ::owlet::platform::pico::PicoUsbSerial;
using ::owlet::tmc2209::Microsteps;

// ── Board ───────────────────────────────────────────────────────────────────

constexpr unsigned kPanStepPin = 2;
constexpr unsigned kPanDirPin = 3;
constexpr unsigned kTiltStepPin = 7; // as built: the reverse of the pan pair
constexpr unsigned kTiltDirPin = 6;
constexpr unsigned kEnablePin = 8;       // both drivers, active low
constexpr unsigned kMs1Pin = 12;         // both drivers
constexpr unsigned kMs2Pin = 13;         // both drivers
constexpr unsigned kActivityLedPin = 22; // sinks the LED: low = lit

// ── Mechanism ───────────────────────────────────────────────────────────────

constexpr uint32_t kFullStepsPerRev = 200; // 1.8° motors
constexpr Microsteps kMicrosteps = Microsteps::k16;
constexpr float kBeltReduction = 3.0f; // 20T motor pulley : 60T output

// Flip if an axis turns the wrong way: pan + must be right, tilt + up.
// Not yet confirmed on the reference head.
constexpr bool kPanReversed = false;
constexpr bool kTiltReversed = false;

// Deliberately gentle: proven on a 5 V bench supply up to 30 °/s, with no
// lost steps. Raise them once the head runs from 12 V; clients read the
// limits with $CAPS and adapt.
constexpr float kMaxSpeedDegPerSec = 120.0f;
constexpr float kAccelDegPerSec2 = 480.0f;

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

} // namespace

int main()
{
    // Driver outputs first, at their off levels: the motors stay limp until
    // the PC sends ENABLE.
    static PicoDigitalOutput enableLine(kEnablePin, PicoDigitalOutput::Polarity::ActiveLow);
    static PicoDigitalOutput ms1(kMs1Pin, PicoDigitalOutput::Polarity::ActiveHigh);
    static PicoDigitalOutput ms2(kMs2Pin, PicoDigitalOutput::Polarity::ActiveHigh);
    static PicoDigitalOutput activityLed(kActivityLedPin, PicoDigitalOutput::Polarity::ActiveLow);
    ::owlet::tmc2209::selectMicrosteps(ms1, ms2, kMicrosteps);

    static PicoUsbSerial usb;
    (void)usb.init(); // without USB there is no one to report a failure to

    // A generator that failed to start refuses every burst, so the axis just
    // never moves and the PC sees a head that doesn't arrive.
    static PicoStepProgram stepProgram(pio0);
    static PicoStepGenerator panSteps(stepProgram, 0, kPanStepPin, kPanDirPin);
    static PicoStepGenerator tiltSteps(stepProgram, 1, kTiltStepPin, kTiltDirPin);
    (void)stepProgram.load();
    (void)panSteps.init();
    (void)tiltSteps.init();

    static SharedEnable enable(enableLine);
    static SharedEnable::User panEnable(enable);
    static SharedEnable::User tiltEnable(enable);
    static StepperAxis pan(panSteps, panEnable, kPanAxis);
    static StepperAxis tilt(tiltSteps, tiltEnable, kTiltAxis);
    static ::owlet::core::PanTilt head(pan, tilt);

    static PicoFlashCalibrationStore calStore;
    static ::owlet::core::Calibrator calibrator(
        pan,
        tilt,
        head,
        calStore,
        ::owlet::core::CalibrationRecord{pan.defaultCalibration(), tilt.defaultCalibration()});
    calibrator.begin();

    // Protocols are plug-ins: to add your own, construct it here and combine
    // them with core::ProtocolMux (docs/DESIGN.md §4.4).
    static PicoClock clock;
    static ::owlet::text::TextProtocol protocol(
        usb, head, calibrator, clock, ::owlet::core::kFirmwareVersion);
    static ::owlet::core::ActivityIndicator motion(head, head, activityLed);
    static ::owlet::core::ControlLoop loop(usb, motion, protocol);

    loop.begin(clock.nowMs());
    absolute_time_t wake = get_absolute_time();
    for (;;)
    {
        loop.service(clock.nowMs());
        wake = delayed_by_us(wake, kLoopPeriodUs);
        // After a long stall (a flash erase), start a fresh schedule instead
        // of spinning through the ticks that were missed.
        if (absolute_time_diff_us(get_absolute_time(), wake) < 0)
        {
            wake = get_absolute_time();
        }
        sleep_until(wake);
    }
}
