# Step/dir stepper axis (hal::IAxis) with an acceleration-limited motion
# profile, plus helpers for standalone TMC2209 modules.

set(OWLET_STEPPER_SRCS
    src/StepperAxis.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_STEPPER_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_hal
    )
else()
    add_library(owlet_stepper STATIC ${OWLET_STEPPER_SRCS})
    add_library(owlet::stepper ALIAS owlet_stepper)
    target_compile_features(owlet_stepper PUBLIC cxx_std_17)
    target_include_directories(owlet_stepper PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_stepper PUBLIC owlet_hal)
endif()
