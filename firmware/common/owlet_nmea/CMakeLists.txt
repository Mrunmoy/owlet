# owlet native protocol ("$BODY*HH", NMEA 0183 framing): codec + the
# NmeaProtocol IProtocol implementation. Pure C++17, no heap.

set(OWLET_NMEA_SRCS
    src/Protocol.cpp
    src/NmeaProtocol.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_NMEA_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_core owlet_hal
    )
else()
    add_library(owlet_nmea STATIC ${OWLET_NMEA_SRCS})
    add_library(owlet::nmea ALIAS owlet_nmea)
    target_include_directories(owlet_nmea PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_nmea PUBLIC owlet_core owlet_hal)
endif()
