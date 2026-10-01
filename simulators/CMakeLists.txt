# Register-level hardware simulators for integration tests.
# Host-only: this subdirectory is only added when OWLET_BUILD_TESTS=ON, so the
# target is never produced for the ESP-IDF build.

add_library(owlet_simulators STATIC
    src/VirtualI2CBus.cpp
    src/PCA9685Simulator.cpp
)
target_include_directories(owlet_simulators PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(owlet_simulators PUBLIC
    owlet_hal
    owlet_pca9685
)

add_subdirectory(tests)
