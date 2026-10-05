# drivers/ aggregates the HAL and the motor drivers: the PCA9685 + logical
# servo, and the step/dir stepper axis.
#
# Directory names carry the `owlet_` prefix so each directory is usable
# directly as an ESP-IDF component (whose name equals its dirname).

add_subdirectory(owlet_hal)
add_subdirectory(owlet_pca9685)
add_subdirectory(owlet_servo)
add_subdirectory(owlet_stepper)
