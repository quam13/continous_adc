idf_component_register(
    SRCS "continuous_read_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_adc    # for the ADC continuous and calibration APIs
        driver     # for driver/gpio.h
)