# PCA9685 I2C PWM driver. Platform-agnostic C++17.
#
# Dual-mode: builds as an ESP-IDF component when ESP_PLATFORM is set, and as
# a host static library otherwise. Both consume the shared owlet_hal
# interfaces for I2C + delay.

set(OWLET_PCA9685_SRCS
    src/PCA9685.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS        ${OWLET_PCA9685_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES    owlet_hal
    )
else()
    add_library(owlet_pca9685 STATIC ${OWLET_PCA9685_SRCS})
    add_library(owlet::pca9685 ALIAS owlet_pca9685)
    target_compile_features(owlet_pca9685 PUBLIC cxx_std_17)
    target_include_directories(owlet_pca9685 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_pca9685 PUBLIC owlet_hal)
endif()
