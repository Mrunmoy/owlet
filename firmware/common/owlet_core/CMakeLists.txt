# Device core: motion model (PanTilt), the protocol-agnostic ControlLoop, and
# the interfaces protocols plug into (IPanTilt, IProtocol, ProtocolMux).
# Depends on no protocol: protocols depend on it.

set(OWLET_CORE_SRCS
    src/AxisHoming.cpp
    src/EventBus.cpp
    src/PanTilt.cpp
    src/TroubleLog.cpp
    src/ControlLoop.cpp
    src/Calibrator.cpp
    src/CalibrationCodec.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_CORE_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_hal
    )
else()
    add_library(owlet_core STATIC ${OWLET_CORE_SRCS})
    add_library(owlet::core ALIAS owlet_core)
    target_compile_features(owlet_core PUBLIC cxx_std_17)
    target_include_directories(owlet_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_core PUBLIC owlet_hal)
endif()
