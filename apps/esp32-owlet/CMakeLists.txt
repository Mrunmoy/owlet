cmake_minimum_required(VERSION 3.20)

# The shared owlet components (same list a parent project would use when
# consuming owlet as a submodule).
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/owlet-idf-components.cmake)
set(EXTRA_COMPONENT_DIRS ${OWLET_IDF_COMPONENT_DIRS})

# sdkconfig.defaults.local (gitignored) holds per-build overrides such as
# measured servo calibration. Applied on top of sdkconfig.defaults.
set(_OWLET_SDKCONFIG_DEFAULTS "sdkconfig.defaults")
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults.local)
    list(APPEND _OWLET_SDKCONFIG_DEFAULTS "sdkconfig.defaults.local")
endif()
set(SDKCONFIG_DEFAULTS "${_OWLET_SDKCONFIG_DEFAULTS}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(owlet)
