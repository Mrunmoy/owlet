cmake_minimum_required(VERSION 3.20)

# Pull in the tree-wide shared components. Each listed directory is an
# ESP-IDF component (its dirname is the component name — hence the
# `owlet_` prefix to avoid collisions with IDF built-ins like `hal`).
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/owlet_hal
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/owlet_pca9685
    ${CMAKE_CURRENT_LIST_DIR}/../../drivers/owlet_servo
    ${CMAKE_CURRENT_LIST_DIR}/../../firmware/common/owlet_protocol
    ${CMAKE_CURRENT_LIST_DIR}/../../firmware/common/owlet_core
    ${CMAKE_CURRENT_LIST_DIR}/../../platform/owlet_esp32
)

# sdkconfig.defaults.local (gitignored) holds per-build overrides such as
# measured servo calibration. Applied on top of sdkconfig.defaults.
set(_OWLET_SDKCONFIG_DEFAULTS "sdkconfig.defaults")
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local)
    list(APPEND _OWLET_SDKCONFIG_DEFAULTS "sdkconfig.defaults.local")
endif()
set(SDKCONFIG_DEFAULTS "${_OWLET_SDKCONFIG_DEFAULTS}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(owlet)
