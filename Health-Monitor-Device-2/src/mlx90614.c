#include "mlx90614.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "mlx90614"

/* RAM register addresses */
#define REG_TA    0x06  /* ambient temperature (die) */
#define REG_TOBJ1 0x07  /* object / surface temperature */

/*
 * MLX90614 SMBus read-word: write 1 command byte, repeated-start, read 3 bytes
 * (data_lo, data_hi, PEC).  The ESP-IDF write_read helper does exactly this.
 *
 * Data format (16-bit):
 *   bit 15  – error flag (1 = invalid)
 *   bits 14:0 – temperature in units of 0.02 K
 */
static esp_err_t read_raw(i2c_port_t port, uint8_t reg, uint16_t *out)
{
    uint8_t buf[3]; /* data_lo, data_hi, pec */
    esp_err_t ret = i2c_master_write_read_device(port, MLX90614_I2C_ADDR,
                                                  &reg, 1,
                                                  buf, sizeof(buf),
                                                  pdMS_TO_TICKS(100));
    if (ret != ESP_OK) return ret;

    if (buf[1] & 0x80) {
        /* Error flag set by sensor */
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out = (uint16_t)(buf[0] | (buf[1] << 8));
    return ESP_OK;
}

static float raw_to_celsius(uint16_t raw)
{
    return (float)raw * 0.02f - 273.15f;
}

/*
 * Verify the sensor is on the bus by reading Ta.
 * The MLX90614 needs no configuration — it starts measuring immediately.
 */
esp_err_t mlx90614_init(i2c_port_t port)
{
    uint16_t raw;
    esp_err_t ret = read_raw(port, REG_TA, &raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Not found on I2C bus (addr 0x%02X): %s",
                 MLX90614_I2C_ADDR, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "MLX90614 found — ambient temp: %.2f C", raw_to_celsius(raw));
    return ESP_OK;
}

/* Read object / surface temperature (register 0x07). */
esp_err_t mlx90614_read_object_temp(i2c_port_t port, float *temp_c)
{
    uint16_t raw;
    esp_err_t ret = read_raw(port, REG_TOBJ1, &raw);
    if (ret != ESP_OK) return ret;
    *temp_c = raw_to_celsius(raw);
    return ESP_OK;
}
