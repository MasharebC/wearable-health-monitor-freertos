#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "ssd1306.h"

#define TAG "MLX90614_OLED"

#ifndef CONFIG_SDA_GPIO
#define CONFIG_SDA_GPIO 21
#endif

#ifndef CONFIG_SCL_GPIO
#define CONFIG_SCL_GPIO 22
#endif

#define I2C_PORT I2C_NUM_0
#define I2C_HZ 100000
#define I2C_TIMEOUT_MS 200

#define OLED_ADDR 0x3C

#define MLX_ADDR 0x5A
#define MLX_REG_TA 0x06
#define MLX_REG_TOBJ1 0x07

#define OLED_UPDATE_MS 1000
#define SMA_WINDOW 5
#define FEVER_THRESHOLD_C 38.0f
#define UNIT_TOGGLE_GPIO GPIO_NUM_0
#define STATUS_LED_GPIO GPIO_NUM_2

typedef enum {
    UNIT_C = 0,
    UNIT_F,
} temp_unit_t;

typedef struct {
    float values[SMA_WINDOW];
    int index;
    int count;
    float sum;
} moving_avg_t;

static esp_err_t i2c_legacy_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = CONFIG_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void unit_button_init(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << UNIT_TOGGLE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
}

static void status_led_init(void)
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << STATUS_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&conf));
    gpio_set_level(STATUS_LED_GPIO, 0);
}

static esp_err_t mlx_read_word(uint8_t reg, uint16_t *out)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MLX_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MLX_ADDR << 1) | I2C_MASTER_READ, true);

    uint8_t data[3];
    i2c_master_read(cmd, data, 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK); // ignore PEC byte
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        return err;
    }

    *out = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    return ESP_OK;
}

static float mlx_raw_to_celsius(uint16_t raw)
{
    return (raw * 0.02f) - 273.15f;
}

static float c_to_f(float c)
{
    return (c * 9.0f / 5.0f) + 32.0f;
}

static float moving_avg_add(moving_avg_t *avg, float value)
{
    if (avg->count < SMA_WINDOW) {
        avg->values[avg->index] = value;
        avg->sum += value;
        avg->count++;
    } else {
        avg->sum -= avg->values[avg->index];
        avg->values[avg->index] = value;
        avg->sum += value;
    }

    avg->index = (avg->index + 1) % SMA_WINDOW;
    return avg->sum / avg->count;
}

static void oled_line(SSD1306_t *oled, int page, const char *text)
{
    char line[17];
    snprintf(line, sizeof(line), "%-16s", text);
    ssd1306_display_text(oled, page, line, strlen(line), false);
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_legacy_init());
    unit_button_init();
    status_led_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    SSD1306_t oled;
    i2c_device_add(&oled, I2C_PORT, -1, OLED_ADDR);
    ssd1306_init(&oled, 128, 64);
    ssd1306_clear_screen(&oled, false);

    moving_avg_t amb_avg = {0};
    moving_avg_t obj_avg = {0};
    temp_unit_t unit = UNIT_C;
    int last_button_level = 1;
    bool led_blink_state = false;

    char line[24];
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        int button_level = gpio_get_level(UNIT_TOGGLE_GPIO);
        if (button_level == 0 && last_button_level == 1) {
            unit = (unit == UNIT_C) ? UNIT_F : UNIT_C;
            ESP_LOGI(TAG, "Display unit changed to %s", unit == UNIT_C ? "C" : "F");
        }
        last_button_level = button_level;

        uint16_t raw_ta = 0;
        uint16_t raw_to = 0;

        if (mlx_read_word(MLX_REG_TA, &raw_ta) == ESP_OK && mlx_read_word(MLX_REG_TOBJ1, &raw_to) == ESP_OK) {
            float amb_c = mlx_raw_to_celsius(raw_ta);
            float obj_c = mlx_raw_to_celsius(raw_to);

            float amb_smooth_c = moving_avg_add(&amb_avg, amb_c);
            float obj_smooth_c = moving_avg_add(&obj_avg, obj_c);

            float amb_display = (unit == UNIT_C) ? amb_smooth_c : c_to_f(amb_smooth_c);
            float obj_display = (unit == UNIT_C) ? obj_smooth_c : c_to_f(obj_smooth_c);
            const char unit_char = (unit == UNIT_C) ? 'C' : 'F';

            ESP_LOGI(TAG,
                     "Ambient: %.2f C (avg %.2f C) | Object: %.2f C (avg %.2f C)",
                     amb_c,
                     amb_smooth_c,
                     obj_c,
                     obj_smooth_c);

            snprintf(line, sizeof(line), "MLX90614  %c", unit_char);
            oled_line(&oled, 0, line);
            snprintf(line, sizeof(line), "Amb:%6.2f %c", amb_display, unit_char);
            oled_line(&oled, 2, line);
            snprintf(line, sizeof(line), "Obj:%6.2f %c", obj_display, unit_char);
            oled_line(&oled, 4, line);

            if (obj_smooth_c >= FEVER_THRESHOLD_C) {
                oled_line(&oled, 6, "ALERT: FEVER");
                gpio_set_level(STATUS_LED_GPIO, 1); // fever: LED on
            } else {
                oled_line(&oled, 6, "Status: Normal");
                gpio_set_level(STATUS_LED_GPIO, 0); // normal: LED off
            }
            oled_line(&oled, 7, "BTN0: C/F Toggle");
        } else {
            ESP_LOGW(TAG, "MLX90614 read failed");
            snprintf(line, sizeof(line), "MLX90614  %c", unit == UNIT_C ? 'C' : 'F');
            oled_line(&oled, 0, line);
            oled_line(&oled, 2, "Amb: --.--");
            oled_line(&oled, 4, "Obj: --.--");
            oled_line(&oled, 6, "Read Fail");
            oled_line(&oled, 7, "Check wiring");

            led_blink_state = !led_blink_state;
            gpio_set_level(STATUS_LED_GPIO, led_blink_state ? 1 : 0); // read fail: blink
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OLED_UPDATE_MS));
    }
}
