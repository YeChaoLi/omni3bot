idf_component_register(SRCS "main.c" "esp_hid_gap.c"
                       INCLUDE_DIRS ""
                       REQUIRES spi_flash esp_hid nvs_flash esp_driver_ledc esp_driver_gpio)

