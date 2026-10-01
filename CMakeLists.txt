cmake_minimum_required(VERSION 3.20)
project(owlet LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Options ──────────────────────────────────────────────────────────────────
option(OWLET_BUILD_TESTS "Build host unit tests with Google Test" ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
endif()

# ── Host-buildable libraries ─────────────────────────────────────────────────
# drivers/ and firmware/common/ are platform-agnostic and compile on the host
# for unit testing. The ESP-IDF firmware build lives under apps/esp32-owlet
# and is invoked separately through idf.py (see build.py --esp).

add_subdirectory(drivers)
add_subdirectory(firmware/common)

# ── Tests + host tools ───────────────────────────────────────────────────────
if(OWLET_BUILD_TESTS)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_subdirectory(simulators)
    add_subdirectory(apps/host-sim)   # before tests: python_client needs owlet_sim
    add_subdirectory(tests)
endif()
