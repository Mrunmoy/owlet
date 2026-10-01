idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        owlet_core
        owlet_nmea
        owlet_pca9685
        owlet_esp32
)
