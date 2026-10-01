# owlet native text protocol ("$BODY*HH" lines): codec + the
# TextProtocol IProtocol implementation. Pure C++17, no heap.

set(OWLET_TEXT_SRCS
    src/Protocol.cpp
    src/TextProtocol.cpp
)

if(ESP_PLATFORM)
    idf_component_register(
        SRCS         ${OWLET_TEXT_SRCS}
        INCLUDE_DIRS "include"
        REQUIRES     owlet_core owlet_hal
    )
else()
    add_library(owlet_text STATIC ${OWLET_TEXT_SRCS})
    add_library(owlet::text ALIAS owlet_text)
    target_compile_features(owlet_text PUBLIC cxx_std_17)
    target_include_directories(owlet_text PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(owlet_text PUBLIC owlet_core owlet_hal)
endif()
