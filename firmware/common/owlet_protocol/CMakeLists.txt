# owlet wire protocol: framing, checksum, request parsing, reply formatting.
# Pure C++17, no heap, no dependencies beyond the standard library.

set(OWLET_PROTOCOL_SRCS
    src/Protocol.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_PROTOCOL_SRCS}
        INCLUDE_DIRS "include"
    )
else()
    add_library(owlet_protocol STATIC ${OWLET_PROTOCOL_SRCS})
    target_include_directories(owlet_protocol PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
