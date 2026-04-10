idf_component_register(SRCS "main.c" "mlx90614.c" "ssd1306.c" "max30102.c"
                      INCLUDE_DIRS "."
                      REQUIRES bt nvs_flash driver esp_timer)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
