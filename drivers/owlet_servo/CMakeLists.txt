# Logical 180° servo. Wraps an IServoChannel with calibration,
# piecewise angle↔pulse mapping, and time-based deg/s slew.

set(OWLET_SERVO_SRCS
    src/Servo.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_SERVO_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_hal
    )
else()
    add_library(owlet_servo STATIC ${OWLET_SERVO_SRCS})
    add_library(owlet::servo ALIAS owlet_servo)
    target_include_directories(owlet_servo PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_servo PUBLIC owlet_hal)
endif()
