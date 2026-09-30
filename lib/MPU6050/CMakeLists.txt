idf_component_register(SRCS "MPU6050.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c esp_driver_gpio)
