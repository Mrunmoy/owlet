# Pure-virtual HAL interfaces (header-only).

add_library(owlet_hal INTERFACE)
add_library(owlet::hal ALIAS owlet_hal)
target_compile_features(owlet_hal INTERFACE cxx_std_17)
target_include_directories(owlet_hal INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
