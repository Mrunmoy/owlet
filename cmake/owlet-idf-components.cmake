# ESP-IDF component directories that make up owlet.
#
# In a parent ESP-IDF project (owlet as a submodule):
#
#   include(${CMAKE_CURRENT_LIST_DIR}/owlet/cmake/owlet-idf-components.cmake)
#   list(APPEND EXTRA_COMPONENT_DIRS ${OWLET_IDF_COMPONENT_DIRS})
#
# then REQUIRES owlet_core owlet_text owlet_pca9685 owlet_servo owlet_esp32 as
# needed (owlet_stepper for a step/dir head).

get_filename_component(_OWLET_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(OWLET_IDF_COMPONENT_DIRS
    ${_OWLET_ROOT}/drivers/owlet_hal
    ${_OWLET_ROOT}/drivers/owlet_pca9685
    ${_OWLET_ROOT}/drivers/owlet_servo
    ${_OWLET_ROOT}/drivers/owlet_stepper
    ${_OWLET_ROOT}/firmware/common/owlet_core
    ${_OWLET_ROOT}/firmware/common/owlet_text
    ${_OWLET_ROOT}/platform/owlet_esp32
)
