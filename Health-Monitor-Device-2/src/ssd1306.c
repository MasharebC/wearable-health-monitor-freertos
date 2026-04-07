#include "ssd1306.h"
#include "font5x7.h"

#include <string.h>

static uint8_t framebuf[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static esp_err_t send_cmd(i2c_port_t port, uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(port, SSD1306_I2C_ADDR, buf, 2,
                                      pdMS_TO_TICKS(100));
}

esp_err_t ssd1306_init(i2c_port_t port)
{
    static const uint8_t init_cmds[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock divide ratio */
        0xA8, 0x3F, /* multiplex ratio = 63 (64 lines) */
        0xD3, 0x00, /* display offset = 0 */
        0x40,       /* start line = 0 */
        0x8D, 0x14, /* enable charge pump */
        0x20, 0x00, /* horizontal addressing mode */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction */
        0xDA, 0x12, /* COM pins config */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* pre-charge period */
        0xDB, 0x40, /* VCOMH deselect level */
        0xA4,       /* display from RAM */
        0xA6,       /* normal (not inverted) */
        0xAF,       /* display on */
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        esp_err_t ret = send_cmd(port, init_cmds[i]);
        if (ret != ESP_OK) return ret;
    }

    ssd1306_clear();
    return ssd1306_update(port);
}

void ssd1306_clear(void)
{
    memset(framebuf, 0, sizeof(framebuf));
}

void ssd1306_draw_string(int x, int y, const char *str)
{
    for (; *str && x + 5 <= SSD1306_WIDTH; str++, x += 6) {
        int idx = (*str < 32 || *str > 126) ? 0 : (*str - 32);
        for (int col = 0; col < 5; col++) {
            uint8_t glyph_col = font5x7[idx * 5 + col];
            for (int bit = 0; bit < 7; bit++) {
                if (glyph_col & (1 << bit)) {
                    int py = y + bit;
                    if (py >= 0 && py < SSD1306_HEIGHT) {
                        framebuf[(py / 8) * SSD1306_WIDTH + (x + col)] |=
                            (1 << (py % 8));
                    }
                }
            }
        }
    }
}

esp_err_t ssd1306_update(i2c_port_t port)
{
    /* Set column and page address to cover the full display */
    send_cmd(port, 0x21); send_cmd(port, 0); send_cmd(port, 127);
    send_cmd(port, 0x22); send_cmd(port, 0); send_cmd(port, 7);

    /* Send framebuffer in chunks (I2C max payload ~128 bytes is safe) */
    for (int i = 0; i < (int)sizeof(framebuf); i += 128) {
        uint8_t buf[129];
        buf[0] = 0x40; /* data byte prefix */
        int chunk = sizeof(framebuf) - i;
        if (chunk > 128) chunk = 128;
        memcpy(&buf[1], &framebuf[i], chunk);
        esp_err_t ret = i2c_master_write_to_device(port, SSD1306_I2C_ADDR,
                                                    buf, chunk + 1,
                                                    pdMS_TO_TICKS(100));
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}
