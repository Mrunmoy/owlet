# Device behaviour: PanTilt motion model + CommandProcessor protocol executor.
# Platform-agnostic and host-tested; the ESP app only wires it up.

set(OWLET_CORE_SRCS
    src/PanTilt.cpp
    src/CommandProcessor.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_CORE_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_hal owlet_servo owlet_protocol
    )
else()
    add_library(owlet_core STATIC ${OWLET_CORE_SRCS})
    target_include_directories(owlet_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_core PUBLIC owlet_hal owlet_servo owlet_protocol)
endif()
