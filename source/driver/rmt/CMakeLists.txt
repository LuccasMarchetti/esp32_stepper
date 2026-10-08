idf_component_register(
    SRCS 
       
        "rmt_stepper_encoder.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        esp_driver_rmt
        esp_driver_gpio
)