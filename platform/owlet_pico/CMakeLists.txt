# Raspberry Pi Pico (RP2040) implementations of the owlet HAL, on pico-sdk.
#
# An INTERFACE library, the pico-sdk way: the sources compile as part of the
# application, so they see the same SDK configuration (stdio, board) it does.

add_library(owlet_pico INTERFACE)
add_library(owlet::pico ALIAS owlet_pico)

target_sources(owlet_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoClock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoDelay.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoDigitalOutput.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoFlashCalibrationStore.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoStepGenerator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PicoUsbSerial.cpp
)
target_include_directories(owlet_pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(owlet_pico INTERFACE
    owlet::hal
    owlet::core
    pico_stdlib
    pico_stdio_usb
    hardware_clocks
    hardware_flash
    hardware_gpio
    hardware_pio
    hardware_sync
)
