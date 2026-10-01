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

owlet_add_test(owlet_smoke_test          test_smoke.cpp)
owlet_add_test(owlet_pca9685_driver_test test_pca9685_driver.cpp owlet_pca9685 owlet_simulators)
owlet_add_test(owlet_servo_test          test_servo.cpp owlet_servo)
owlet_add_test(owlet_protocol_test       test_protocol.cpp owlet_protocol)
owlet_add_test(owlet_pan_tilt_test       test_pan_tilt.cpp owlet_core)
owlet_add_test(owlet_command_processor_test test_command_processor.cpp owlet_core)
owlet_add_test(owlet_integration_test    test_integration.cpp owlet_core owlet_pca9685 owlet_simulators)

# The protocol vectors file is shared with the Python client tests.
target_compile_definitions(owlet_protocol_test PRIVATE
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
