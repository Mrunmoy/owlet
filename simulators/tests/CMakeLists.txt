include(GoogleTest)

add_executable(owlet_virtual_i2c_bus_test
    test_virtual_i2c_bus.cpp
)
target_link_libraries(owlet_virtual_i2c_bus_test PRIVATE
    owlet_simulators
    GTest::gtest_main
    GTest::gmock
)
gtest_discover_tests(owlet_virtual_i2c_bus_test)

add_executable(owlet_pca9685_simulator_test
    test_pca9685_simulator.cpp
)
target_link_libraries(owlet_pca9685_simulator_test PRIVATE
    owlet_simulators
    GTest::gtest_main
    GTest::gmock
)
gtest_discover_tests(owlet_pca9685_simulator_test)
