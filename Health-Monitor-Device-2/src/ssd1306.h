#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#define SSD1306_I2C_ADDR 0x3C
#define SSD1306_WIDTH    128
#define SSD1306_HEIGHT   64

esp_err_t ssd1306_init(i2c_port_t port);
void ssd1306_clear(void);
void ssd1306_draw_string(int x, int y, const char *str);
esp_err_t ssd1306_update(i2c_port_t port);
