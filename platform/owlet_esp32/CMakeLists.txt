# ESP-IDF HAL implementation. The only HAL layer allowed to include ESP-IDF.
# Esp32I2CBus (driver/i2c_master.h API), Esp32Delay, Esp32UsbSerial.

if(ESP_PLATFORM)
    idf_component_register(
        SRCS
            src/Esp32I2CBus.cpp
            src/Esp32Delay.cpp
            src/Esp32UsbSerial.cpp
        INCLUDE_DIRS
            "include"
        REQUIRES
            owlet_hal
            esp_driver_i2c
            esp_driver_gpio
            esp_driver_usb_serial_jtag
            esp_rom
            freertos
    )
endif()
# Host build ignores this directory by design.
