// owlet firmware entry point for the Raspberry Pi Pico stepper head:
// composition root only.
//
// Builds the platform objects, injects them into the platform-agnostic core,
// and runs the service loop. All behaviour lives in firmware/common and
// drivers/ (host-tested); nothing here makes decisions. owlet_sim runs the
// same core and protocol on a PC.
//
// Wiring: docs/wiring/pico-tmc2209/.

#include "board.hpp"

#include "owlet/core/ActivityIndicator.hpp"
#include "owlet/core/AxisEventRelay.hpp"
#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/EventBus.hpp"
#include "owlet/core/HeadHoming.hpp"
#include "owlet/core/LimitTrip.hpp"
#include "owlet/core/HeadDiagnostics.hpp"
#include "owlet/core/Heartbeat.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Poller.hpp"
#include "owlet/core/TroubleLog.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/platform/pico/PicoClock.hpp"
#include "owlet/platform/pico/PicoDigitalInput.hpp"
#include "owlet/platform/pico/PicoDigitalOutput.hpp"
#include "owlet/platform/pico/PicoFlashCalibrationStore.hpp"
#include "owlet/platform/pico/PicoHalfDuplexUart.hpp"
#include "owlet/platform/pico/PicoInterruptLock.hpp"
#include "owlet/platform/pico/PicoStepGenerator.hpp"
#include "owlet/platform/pico/PicoUsbSerial.hpp"
#include "owlet/stepper/SharedEnable.hpp"
#include "owlet/stepper/StepperAxis.hpp"
#include "owlet/stepper/Tmc2209.hpp"
#include "owlet/stepper/Tmc2209Monitor.hpp"
#include "owlet/stepper/Tmc2209Standalone.hpp"
#include "owlet/text/TextProtocol.hpp"

#include "hardware/pio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <cstdint>

namespace
{

using ::owlet::SharedEnable;
using ::owlet::StepperAxis;
using ::owlet::platform::pico::PicoClock;
using ::owlet::platform::pico::PicoDigitalInput;
using ::owlet::platform::pico::PicoDigitalOutput;
using ::owlet::platform::pico::PicoFlashCalibrationStore;
using ::owlet::platform::pico::PicoHalfDuplexUart;
using ::owlet::platform::pico::PicoInterruptLock;
using ::owlet::platform::pico::PicoStepGenerator;
using ::owlet::platform::pico::PicoStepProgram;
using ::owlet::platform::pico::PicoUsbSerial;
using ::owlet::tmc2209::Microsteps;

// The head itself (pins, motors, speeds, limits) is described in board.hpp.
using namespace board; // NOLINT(google-build-using-namespace): the composition root reads better unqualified

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

    static PicoDigitalOutput heartbeatLed(kHeartbeatLedPin,
                                          PicoDigitalOutput::Polarity::ActiveHigh);

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
    // Whatever notices a change posts it here; listeners are attached below,
    // in the order they should hear.
    static PicoClock clock;
    static PicoInterruptLock eventLock;
    static ::owlet::core::StaticEventBus<kEventQueueDepth, kEventListeners> events(eventLock, clock);

    StepperAxis::LimitSwitches panLimits;
    StepperAxis::LimitSwitches tiltLimits;
    if (kLimitSwitchesFitted)
    {
        static PicoDigitalInput panMin(kPanMinSwitchPin, kSwitchPolarity, kSwitchPull);
        static PicoDigitalInput panMax(kPanMaxSwitchPin, kSwitchPolarity, kSwitchPull);
        static PicoDigitalInput tiltMin(kTiltMinSwitchPin, kSwitchPolarity, kSwitchPull);
        static PicoDigitalInput tiltMax(kTiltMaxSwitchPin, kSwitchPolarity, kSwitchPull);
        panLimits = StepperAxis::LimitSwitches{&panMin, &panMax};
        tiltLimits = StepperAxis::LimitSwitches{&tiltMin, &tiltMax};

        // A switch closing cuts that axis's pulses from its interrupt.
        using ::owlet::core::LimitEnd;
        using ::owlet::core::LimitTrip;
        static LimitTrip panMinTrip(panSteps, events, 0, LimitEnd::Min);
        static LimitTrip panMaxTrip(panSteps, events, 0, LimitEnd::Max);
        static LimitTrip tiltMinTrip(tiltSteps, events, 1, LimitEnd::Min);
        static LimitTrip tiltMaxTrip(tiltSteps, events, 1, LimitEnd::Max);
        panMin.onActive(panMinTrip);
        panMax.onActive(panMaxTrip);
        tiltMin.onActive(tiltMinTrip);
        tiltMax.onActive(tiltMaxTrip);
    }
    static StepperAxis pan(panSteps, panEnable, kPanAxis, panLimits);
    static StepperAxis tilt(tiltSteps, tiltEnable, kTiltAxis, tiltLimits);

    ::owlet::core::IHoming* homing = nullptr;
    if (kLimitSwitchesFitted)
    {
        using ::owlet::core::AxisHoming;
        static AxisHoming panHoming(pan, pan, *panLimits.min, panLimits.max, AxisHoming::Config{});
        static AxisHoming tiltHoming(tilt, tilt, *tiltLimits.min, tiltLimits.max, AxisHoming::Config{});
        static ::owlet::core::HeadHoming headHoming(panHoming, tiltHoming);
        homing = &headHoming;
    }
    static ::owlet::core::LoopStats loopStats;
    static ::owlet::core::TroubleLog trouble;
    (void)events.attach(trouble);
    static ::owlet::core::PanTilt head(pan, tilt, &events, homing);

    static PicoFlashCalibrationStore calStore;
    static ::owlet::core::Calibrator calibrator(
        pan,
        tilt,
        head,
        calStore,
        ::owlet::core::CalibrationRecord{pan.defaultCalibration(), tilt.defaultCalibration()});
    calibrator.begin();

    // The drivers' UARTs. A link that failed to start, or a driver that does
    // not answer, just leaves that axis in standalone mode with no diagnostics.
    static PicoHalfDuplexUart panLink(uart1, kPanUartTxPin, kPanUartRxPin);
    static PicoHalfDuplexUart tiltLink(uart0, kTiltUartTxPin, kTiltUartRxPin);
    (void)panLink.init();
    (void)tiltLink.init();
    static ::owlet::Tmc2209 panDriver(panLink, kDriver);
    static ::owlet::Tmc2209 tiltDriver(tiltLink, kDriver);
    static ::owlet::core::AxisEventRelay panDriverEvents(0, events);
    static ::owlet::core::AxisEventRelay tiltDriverEvents(1, events);
    static ::owlet::Tmc2209Monitor panMonitor(panDriver, 0, &panDriverEvents);
    static ::owlet::Tmc2209Monitor tiltMonitor(
        tiltDriver, ::owlet::Tmc2209Monitor::kPollPeriodMs / 2, &tiltDriverEvents);
    static ::owlet::core::HeadDiagnostics diagnostics(&panMonitor, &tiltMonitor);
    static ::owlet::core::Heartbeat heartbeat(heartbeatLed);

    // Protocols are plug-ins: to add your own, construct it here and combine
    // them with core::ProtocolMux (docs/DESIGN.md §4.4).
    static ::owlet::text::TextProtocol protocol(
        usb, head, calibrator, clock, ::owlet::core::kFirmwareVersion, &diagnostics, &events, &trouble, &loopStats);
    (void)events.attach(protocol);
    static ::owlet::core::ActivityIndicator indicator(head, head, activityLed);
    static ::owlet::core::Poller<3> motion(indicator, {&panMonitor, &tiltMonitor, &heartbeat});
    static ::owlet::core::ControlLoop loop(usb, motion, protocol, &events);

    loop.watchTiming(loopStats, kLoopPeriodUs / 1000u);
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
