idf_component_register(SRCS "main.cpp" "esp_hid_gap.c"
                       INCLUDE_DIRS ""
                       REQUIRES spi_flash esp_hid nvs_flash esp_driver_ledc esp_driver_gpio eigen)

# Set C++ standard
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Add C++ specific flags
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-rtti -fno-exceptions)

