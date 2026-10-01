// owlet firmware entry point — composition root only.
//
// Builds the platform objects, injects them into the platform-agnostic core,
// and runs the 100 Hz service loop. All behaviour lives in
// firmware/common/owlet_core (host-tested); nothing here makes decisions.

#include "owlet/core/ControlLoop.hpp"
#include "owlet/core/PanTilt.hpp"
#include "owlet/core/Version.hpp"
#include "owlet/nmea/NmeaProtocol.hpp"
#include "owlet/pca9685/PCA9685.hpp"
#include "owlet/platform/esp32/Esp32Delay.hpp"
#include "owlet/platform/esp32/Esp32I2CBus.hpp"
#include "owlet/platform/esp32/Esp32UsbSerial.hpp"
#include "owlet/servo/Servo.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <cstdint>

namespace
{

using ::owlet::PCA9685;
using ::owlet::Servo;
using ::owlet::platform::esp32::Esp32Delay;
using ::owlet::platform::esp32::Esp32I2CBus;
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

constexpr Servo::Calibration kPanCal = makeCalibration(
    CONFIG_OWLET_PAN_PULSE_MIN_US, CONFIG_OWLET_PAN_PULSE_CENTER_US, CONFIG_OWLET_PAN_PULSE_MAX_US,
    CONFIG_OWLET_PAN_LIMIT_MIN_DEG, CONFIG_OWLET_PAN_LIMIT_MAX_DEG);

constexpr Servo::Calibration kTiltCal = makeCalibration(
    CONFIG_OWLET_TILT_PULSE_MIN_US, CONFIG_OWLET_TILT_PULSE_CENTER_US,
    CONFIG_OWLET_TILT_PULSE_MAX_US, CONFIG_OWLET_TILT_LIMIT_MIN_DEG,
    CONFIG_OWLET_TILT_LIMIT_MAX_DEG);

uint32_t millis()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

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
    static ::owlet::core::PanTilt head(pan, tilt);
    // Protocols are plug-ins: to add your own, construct it here and combine
    // them with core::ProtocolMux (docs/DESIGN.md §4.4).
    static ::owlet::nmea::NmeaProtocol protocol(usb, head, ::owlet::core::kFirmwareVersion);
    static ::owlet::core::ControlLoop  loop(usb, head, protocol);

    loop.begin(millis());
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        loop.service(millis());
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kLoopPeriodMs));
    }
}
