idf_component_register(
    SRCS 
        "NvsStorage.cpp"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        nvs_flash
)