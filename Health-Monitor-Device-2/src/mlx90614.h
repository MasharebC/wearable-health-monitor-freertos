#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#define MLX90614_I2C_ADDR 0x5A

esp_err_t mlx90614_init(i2c_port_t port);
esp_err_t mlx90614_read_object_temp(i2c_port_t port, float *temp_c);
