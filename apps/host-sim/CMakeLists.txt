# owlet_sim: the real firmware core behind a Linux pseudo-terminal, so the PC
# client can be exercised with no hardware attached. Host-only tool.

if(NOT UNIX)
    return()
endif()

add_executable(owlet_sim main.cpp)
target_link_libraries(owlet_sim PRIVATE
    owlet::text
    owlet::pca9685
    owlet_simulators
)
