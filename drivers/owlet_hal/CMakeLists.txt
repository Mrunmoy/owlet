# Pure-virtual HAL interfaces (header-only).
# II2CBus, IDelayProvider, IServoChannel live under include/owlet/hal/.
#
# Dual build: appears as an ESP-IDF component under ESP_PLATFORM, and as a
# host INTERFACE library otherwise.

if(ESP_PLATFORM)
    idf_component_register(
        INCLUDE_DIRS "include"
    )
else()
    add_library(owlet_hal INTERFACE)
    target_include_directories(owlet_hal INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
