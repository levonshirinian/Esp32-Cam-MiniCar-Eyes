idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp32-camera nvs_flash esp_psram esp_hw_support
             esp_driver_uart esp_driver_gpio
)