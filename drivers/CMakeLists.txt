# drivers/ aggregates the HAL, the PCA9685 driver, and the logical servo.
#
# Directory names carry the `owlet_` prefix so each directory is usable
# directly as an ESP-IDF component (whose name equals its dirname).

add_subdirectory(owlet_hal)
add_subdirectory(owlet_pca9685)
add_subdirectory(owlet_servo)
