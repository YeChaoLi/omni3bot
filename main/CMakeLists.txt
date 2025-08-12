idf_component_register(SRCS "system_monitor.cpp" "motion_manager.cpp" "display_manager.cpp" "sensor_manager.cpp" "radio_manager.cpp" "ble_manager.cpp" "robot_status.cpp" "main.cpp" "esp_hid_gap.c"
                       INCLUDE_DIRS "." "../managed_components/espressif__eigen/eigen"
                       REQUIRES spi_flash esp_hid nvs_flash esp_driver_ledc esp_driver_gpio eigen ESP_CRSF)

# Set C++ standard
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Add C++ specific flags
target_compile_options(${COMPONENT_LIB} PRIVATE -fno-rtti -fno-exceptions)

