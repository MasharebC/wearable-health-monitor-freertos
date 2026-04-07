idf_component_register(SRCS "main.c" "mlx90614.c" "ssd1306.c"
                      INCLUDE_DIRS "."
                      REQUIRES bt nvs_flash driver)
