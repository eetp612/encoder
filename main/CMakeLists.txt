idf_component_register(
    SRCS "motor-driver.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        micro_ros_espidf_component
        esp_driver_ledc
        esp_driver_gpio
)