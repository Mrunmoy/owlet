include(GoogleTest)

# Header-only test doubles shared across tests.
add_library(owlet_mocks INTERFACE)
target_include_directories(owlet_mocks INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)
target_link_libraries(owlet_mocks INTERFACE
    owlet_hal
    GTest::gmock
)

function(owlet_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ${ARGN} owlet_mocks GTest::gtest_main GTest::gmock)
    gtest_discover_tests(${name})
endfunction()

owlet_add_test(owlet_smoke_test             test_smoke.cpp)
owlet_add_test(owlet_pca9685_driver_test    test_pca9685_driver.cpp owlet::pca9685 owlet_simulators)
owlet_add_test(owlet_servo_test             test_servo.cpp owlet::servo)
owlet_add_test(owlet_text_codec_test        test_text_codec.cpp owlet::text)
owlet_add_test(owlet_stepper_axis_test      test_stepper_axis.cpp owlet::core owlet::stepper)
owlet_add_test(owlet_pan_tilt_test          test_pan_tilt.cpp owlet::core owlet::servo)
owlet_add_test(owlet_calibration_test       test_calibration.cpp owlet::core owlet::servo)
owlet_add_test(owlet_activity_indicator_test test_activity_indicator.cpp owlet::core)
owlet_add_test(owlet_protocol_mux_test      test_protocol_mux.cpp owlet::core)
owlet_add_test(owlet_control_loop_test      test_control_loop.cpp owlet::core)
owlet_add_test(owlet_text_protocol_test     test_text_protocol.cpp owlet::text owlet::servo)
owlet_add_test(owlet_integration_test       test_integration.cpp owlet::text owlet::servo owlet::pca9685 owlet_simulators)

# The protocol vectors file is shared with the Python client tests.
target_compile_definitions(owlet_text_codec_test PRIVATE
    OWLET_PROTOCOL_VECTORS="${PROJECT_SOURCE_DIR}/docs/protocol_vectors.txt"
)

# Python client: protocol unit tests + end-to-end against owlet_sim over a pty.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND AND TARGET owlet_sim)
    add_test(NAME python_client
        COMMAND ${Python3_EXECUTABLE} -m unittest discover -s tests
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/tools/python
    )
    set_tests_properties(python_client PROPERTIES
        ENVIRONMENT "OWLET_SIM=$<TARGET_FILE:owlet_sim>"
        TIMEOUT 60
    )
endif()
