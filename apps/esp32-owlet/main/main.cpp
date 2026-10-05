// owlet firmware entry point — composition root only.
//
// Builds the platform objects, injects them into the platform-agnostic core,
// and runs the 100 Hz service loop. All behaviour lives in
// firmware/common/owlet_core (host-tested); nothing here makes decisions.

#include "owlet/core/Calibrator.hpp"
#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/EventBus.hpp"
#include "owlet/core/TroubleLog.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/text/TextProtocol.hpp"
#include "owlet/pca9685/PCA9685.hpp"
#include "owlet/platform/esp32/Esp32Clock.hpp"
#include "owlet/platform/esp32/Esp32Delay.hpp"
#include "owlet/platform/esp32/Esp32I2CBus.hpp"
#include "owlet/platform/esp32/Esp32NvsCalibrationStore.hpp"
#include "owlet/platform/esp32/Esp32UsbSerial.hpp"
#include "owlet/servo/Servo.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <cstdint>

namespace
{

using ::owlet::PCA9685;
using ::owlet::Servo;
using ::owlet::platform::esp32::Esp32Clock;
using ::owlet::platform::esp32::Esp32Delay;
using ::owlet::platform::esp32::Esp32I2CBus;
using ::owlet::platform::esp32::Esp32NvsCalibrationStore;
using ::owlet::platform::esp32::Esp32UsbSerial;

constexpr const char* kTag          = "owlet";
constexpr uint32_t    kLoopPeriodMs = 10;

constexpr Servo::Calibration makeCalibration(int minUs, int centerUs, int maxUs,
                                             int minDeg, int maxDeg)
{
    Servo::Calibration c;
    c.minPulseUs    = static_cast<uint16_t>(minUs);
    c.centerPulseUs = static_cast<uint16_t>(centerUs);
    c.maxPulseUs    = static_cast<uint16_t>(maxUs);
    c.minDeg        = static_cast<float>(minDeg);
    c.maxDeg        = static_cast<float>(maxDeg);
    return c;
}

// Kconfig values are the *factory* calibration. A calibration saved on the
// device with $CAL,SAVE takes precedence at boot.
constexpr Servo::Calibration kPanCal = makeCalibration(
    CONFIG_OWLET_PAN_PULSE_MIN_US, CONFIG_OWLET_PAN_PULSE_CENTER_US, CONFIG_OWLET_PAN_PULSE_MAX_US,
    CONFIG_OWLET_PAN_LIMIT_MIN_DEG, CONFIG_OWLET_PAN_LIMIT_MAX_DEG);

constexpr Servo::Calibration kTiltCal = makeCalibration(
    CONFIG_OWLET_TILT_PULSE_MIN_US, CONFIG_OWLET_TILT_PULSE_CENTER_US,
    CONFIG_OWLET_TILT_PULSE_MAX_US, CONFIG_OWLET_TILT_LIMIT_MIN_DEG,
    CONFIG_OWLET_TILT_LIMIT_MAX_DEG);

} // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "owlet %.*s", static_cast<int>(::owlet::core::kFirmwareVersion.size()),
             ::owlet::core::kFirmwareVersion.data());

    static Esp32UsbSerial usb;
    if (!usb.init())
    {
        ESP_LOGE(kTag, "USB-Serial-JTAG driver install failed");
        return;
    }

    // A missing or unpowered PCA9685 is reported to the PC (NAK,ENABLE,BUS)
    // rather than halting: the operator is at the PC, not at the log UART.
    static Esp32I2CBus bus;
    Esp32I2CBus::Config i2cCfg;
    i2cCfg.sdaPin = static_cast<gpio_num_t>(CONFIG_OWLET_I2C_SDA_GPIO);
    i2cCfg.sclPin = static_cast<gpio_num_t>(CONFIG_OWLET_I2C_SCL_GPIO);
    if (!bus.init(i2cCfg))
    {
        ESP_LOGE(kTag, "I2C init failed");
    }

    static Esp32Delay delay;
    static PCA9685    pwm(bus, delay);
    if (!pwm.init() || !pwm.setAllIdle())
    {
        ESP_LOGE(kTag, "PCA9685 not responding at 0x40 (SDA=GP%d SCL=GP%d)",
                 CONFIG_OWLET_I2C_SDA_GPIO, CONFIG_OWLET_I2C_SCL_GPIO);
    }
    else
    {
        ESP_LOGI(kTag, "PCA9685 ready @ %lu Hz, servos idle",
                 static_cast<unsigned long>(pwm.frequencyHz()));
    }

    static PCA9685::Channel       panCh  = pwm.channel(CONFIG_OWLET_PAN_CHANNEL);
    static PCA9685::Channel       tiltCh = pwm.channel(CONFIG_OWLET_TILT_CHANNEL);
    static Servo                  pan(panCh, kPanCal);
    static Servo                  tilt(tiltCh, kTiltCal);
    // Nothing on this head posts from an interrupt, and one task owns all
    // of it, so the event queue needs no real lock.
    static Esp32Clock                           clock;
    static ::owlet::hal::NullInterruptLock      eventLock;
    static ::owlet::core::StaticEventBus<16, 2> events(eventLock, clock);
    static ::owlet::core::TroubleLog            trouble;
    (void) events.attach(trouble);
    static ::owlet::core::PanTilt               head(pan, tilt, &events);
    // Protocols are plug-ins: to add your own, construct it here and combine
    // them with core::ProtocolMux (docs/DESIGN.md §4.4).
    static Esp32NvsCalibrationStore calStore;
    if (!calStore.init())
    {
        ESP_LOGE(kTag, "NVS init failed: calibration can't be saved this session");
    }
    static ::owlet::core::Calibrator calibrator(pan, tilt, head, calStore,
                                                ::owlet::core::CalibrationRecord{
                                                    Servo::toAxisCalibration(kPanCal),
                                                    Servo::toAxisCalibration(kTiltCal)});
    calibrator.begin();
    if (!calibrator.factoryWasValid())
    {
        ESP_LOGW(kTag, "Kconfig calibration is invalid; using safe defaults as factory values");
    }
    ESP_LOGI(kTag, "calibration: %s",
             calibrator.source() == ::owlet::core::ICalibration::Source::Saved ? "saved" : "factory");

    static ::owlet::text::TextProtocol protocol(usb, head, calibrator, clock,
                                                ::owlet::core::kFirmwareVersion, nullptr, &events, &trouble);
    (void) events.attach(protocol);
    static ::owlet::core::ControlLoop  loop(usb, head, protocol, &events);

    loop.begin(clock.nowMs());
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        loop.service(clock.nowMs());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kLoopPeriodMs));
    }
}
