# owlet firmware for the Raspberry Pi Pico stepper head (pico-sdk build).
#
#   nix develop
#   ./build.py --pico            # → apps/pico-owlet/build/pico_owlet.uf2
#
# PICO_SDK_PATH comes from the dev shell and must be an SDK *with* submodules
# (TinyUSB), which the plain nixpkgs package is not.

cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set; run inside `nix develop`")
endif()
set(PICO_BOARD pico CACHE STRING "pico-sdk board")
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_owlet LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# The shared, platform-agnostic components build here as ordinary static
# libraries: the same sources the host tests exercise.
get_filename_component(OWLET_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_subdirectory(${OWLET_ROOT}/drivers          owlet_drivers)
add_subdirectory(${OWLET_ROOT}/firmware/common  owlet_common)
add_subdirectory(${OWLET_ROOT}/platform/owlet_pico owlet_pico)

add_executable(pico_owlet main.cpp)

# owlet's own target-side sources get the same strict warnings as the host
# build. Set per file, so the pico-sdk sources compiled into this target are
# left alone.
get_target_property(_OWLET_PICO_SOURCES owlet_pico INTERFACE_SOURCES)
set_source_files_properties(main.cpp ${_OWLET_PICO_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-Wno-sign-conversion"
)
target_link_libraries(pico_owlet PRIVATE
    owlet::text
    owlet::core
    owlet::stepper
    owlet::pico
)

# USB carries the protocol and nothing else; there is no UART console.
pico_enable_stdio_usb(pico_owlet 1)
pico_enable_stdio_uart(pico_owlet 0)
target_compile_definitions(pico_owlet PRIVATE
    # A host that stops reading must not stall the control loop: give up on a
    # write after 5 ms and drop the line.
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000
)

pico_set_program_name(pico_owlet "owlet")
pico_add_extra_outputs(pico_owlet)   # .uf2 for drag-and-drop / picotool
