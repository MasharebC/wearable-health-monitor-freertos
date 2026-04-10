#include "max30102.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "max30102";

/* ── Register map ────────────────────────────────────────────────── */
#define REG_FIFO_WR_PTR  0x04
#define REG_OVF_COUNTER  0x05
#define REG_FIFO_RD_PTR  0x06
#define REG_FIFO_DATA    0x07
#define REG_FIFO_CFG     0x08
#define REG_MODE_CFG     0x09
#define REG_SPO2_CFG     0x0A
#define REG_LED1_PA      0x0C   /* Red LED pulse amplitude  */
#define REG_LED2_PA      0x0D   /* IR  LED pulse amplitude  */
#define REG_PART_ID      0xFF

#define EXPECTED_PART_ID 0x15

/* ── Rolling sample buffer ───────────────────────────────────────── */
/*
 * Effective FIFO output rate = 100 SPS / 4 (averaging) = 25 samples/s.
 * N_SAMPLES = 100  →  4 seconds of data, enough for reliable peak detection
 * across 40–220 BPM and a stable AC/DC ratio for SpO2.
 */
#define N_SAMPLES 100

static uint32_t s_ir [N_SAMPLES];
static uint32_t s_red[N_SAMPLES];
static int      s_n = 0;          /* valid entries currently in buffer */

/* ── I2C helpers ─────────────────────────────────────────────────── */

static esp_err_t reg_write(i2c_port_t port, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(port, MAX30102_I2C_ADDR,
                                      buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t reg_read(i2c_port_t port, uint8_t reg,
                          uint8_t *dst, size_t len)
{
    return i2c_master_write_read_device(port, MAX30102_I2C_ADDR,
                                        &reg, 1, dst, len,
                                        pdMS_TO_TICKS(100));
}

/* ── Initialisation ──────────────────────────────────────────────── */

esp_err_t max30102_init(i2c_port_t port)
{
    uint8_t pid = 0;
    esp_err_t ret = reg_read(port, REG_PART_ID, &pid, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (pid != EXPECTED_PART_ID) {
        /* Many cheap MAX30102 modules ship with clone chips that return a
         * different part ID (e.g. 0x1D) but are register-compatible.
         * Log a warning but continue — if the subsequent config writes
         * succeed the sensor is usable. */
        ESP_LOGW(TAG, "Unexpected part ID 0x%02X (expected 0x%02X) — "
                 "possible clone chip, attempting to configure anyway",
                 pid, EXPECTED_PART_ID);
    }

    /* Software reset — clears FIFO and all config registers */
    ret = reg_write(port, REG_MODE_CFG, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    /*
     * FIFO config: 0x4F
     *   SMP_AVE   [7:5] = 010  → average 4 raw samples per FIFO write
     *   ROLLOVER  [4]   = 1    → oldest sample overwritten when full
     *   A_FULL    [3:0] = 0xF  → almost-full threshold (interrupts unused)
     *
     * At 100 SPS raw with ×4 averaging → 25 effective FIFO writes/s.
     * 32-slot FIFO fills in ~1.28 s, safely above our 1 s read period.
     */
    ret = reg_write(port, REG_FIFO_CFG, 0x4F);
    if (ret != ESP_OK) return ret;

    /* SpO2 mode: both Red (LED1) and IR (LED2) active */
    ret = reg_write(port, REG_MODE_CFG, 0x03);
    if (ret != ESP_OK) return ret;

    /*
     * SpO2 config: 0x27
     *   ADC range [6:5] = 01  → 4096 nA full scale
     *   SR        [4:2] = 001 → 100 raw samples/s
     *   LED PW    [1:0] = 11  → 411 µs pulse, 18-bit ADC resolution
     */
    ret = reg_write(port, REG_SPO2_CFG, 0x27);
    if (ret != ESP_OK) return ret;

    /*
     * LED pulse amplitudes: 0x24 = 36 → 36 × 0.2 mA = 7.2 mA each.
     * Increase (up to 0xFF = 51 mA) if signal appears weak.
     */
    ret = reg_write(port, REG_LED1_PA, 0x24);
    if (ret != ESP_OK) return ret;
    ret = reg_write(port, REG_LED2_PA, 0x24);
    if (ret != ESP_OK) return ret;

    /* Clear FIFO pointers */
    reg_write(port, REG_FIFO_WR_PTR, 0x00);
    reg_write(port, REG_OVF_COUNTER, 0x00);
    reg_write(port, REG_FIFO_RD_PTR, 0x00);

    s_n = 0;
    ESP_LOGI(TAG, "MAX30102 configured (part ID 0x%02X, addr 0x%02X)",
             pid, MAX30102_I2C_ADDR);
    return ESP_OK;
}

/* ── Algorithm helpers ───────────────────────────────────────────── */

static float mean_u32(const uint32_t *data, int n)
{
    double s = 0;
    for (int i = 0; i < n; i++) s += data[i];
    return (float)(s / n);
}

/*
 * Count peaks in `data` that exceed mean + 0.5 × std.
 * Adjacent samples are skipped after each peak to avoid double-counting.
 */
static int count_peaks(const uint32_t *data, int n, float mean)
{
    double var = 0;
    for (int i = 0; i < n; i++) {
        float d = (float)data[i] - mean;
        var += (double)(d * d);
    }
    float threshold = mean + 0.5f * sqrtf((float)(var / n));

    int peaks = 0;
    for (int i = 1; i < n - 1; i++) {
        if ((float)data[i] > threshold &&
            data[i] > data[i - 1] &&
            data[i] > data[i + 1]) {
            peaks++;
            i++;   /* skip next sample to avoid re-triggering on the same peak */
        }
    }
    return peaks;
}

/* ── Public API ──────────────────────────────────────────────────── */

esp_err_t max30102_read_vitals(i2c_port_t port, max30102_result_t *out)
{
    out->hr_valid        = false;
    out->spo2_valid      = false;
    out->heart_rate_bpm  = 0.0f;
    out->spo2_pct        = 0.0f;

    /* ── 1. Read FIFO pointers (3 consecutive registers: WR, OVF, RD) ── */
    uint8_t ptrs[3] = {0};
    esp_err_t ret = reg_read(port, REG_FIFO_WR_PTR, ptrs, 3);
    if (ret != ESP_OK) return ret;

    uint8_t wr  = ptrs[0] & 0x1F;
    uint8_t ovf = ptrs[1];
    uint8_t rd  = ptrs[2] & 0x1F;

    int avail = (wr - rd + 32) % 32;
    if (avail == 0 && ovf > 0) avail = 32;   /* full FIFO after rollover */
    if (avail == 0) return ESP_OK;             /* nothing new yet          */

    /* ── 2. Burst-read FIFO bytes ── */
    /* SpO2 mode: 6 bytes per sample — [Red: 3 B][IR: 3 B], big-endian 18-bit */
    uint8_t raw[32 * 6];
    ret = reg_read(port, REG_FIFO_DATA, raw, (size_t)avail * 6);
    if (ret != ESP_OK) return ret;

    /* ── 3. Parse samples ── */
    uint32_t new_red[32], new_ir[32];
    for (int i = 0; i < avail; i++) {
        const uint8_t *p = raw + i * 6;
        new_red[i] = ((uint32_t)(p[0] & 0x03) << 16) |
                     ((uint32_t) p[1]           <<  8) |
                      (uint32_t) p[2];
        new_ir[i]  = ((uint32_t)(p[3] & 0x03) << 16) |
                     ((uint32_t) p[4]           <<  8) |
                      (uint32_t) p[5];
    }

    /* ── 4. Slide rolling buffer ── */
    if (avail >= N_SAMPLES) {
        /* More new samples than buffer — take the latest N_SAMPLES */
        memcpy(s_ir,  new_ir  + avail - N_SAMPLES, N_SAMPLES * sizeof(uint32_t));
        memcpy(s_red, new_red + avail - N_SAMPLES, N_SAMPLES * sizeof(uint32_t));
        s_n = N_SAMPLES;
    } else if (s_n + avail <= N_SAMPLES) {
        /* Buffer not yet full — append */
        memcpy(s_ir  + s_n, new_ir,  (size_t)avail * sizeof(uint32_t));
        memcpy(s_red + s_n, new_red, (size_t)avail * sizeof(uint32_t));
        s_n += avail;
    } else {
        /* Shift oldest out, append newest */
        int keep = N_SAMPLES - avail;
        memmove(s_ir,  s_ir  + (s_n - keep), (size_t)keep * sizeof(uint32_t));
        memmove(s_red, s_red + (s_n - keep), (size_t)keep * sizeof(uint32_t));
        memcpy(s_ir  + keep, new_ir,  (size_t)avail * sizeof(uint32_t));
        memcpy(s_red + keep, new_red, (size_t)avail * sizeof(uint32_t));
        s_n = N_SAMPLES;
    }

    if (s_n < N_SAMPLES) return ESP_OK;   /* still warming up */

    /* ── 5. Finger-presence check ── */
    /*
     * With 18-bit ADC and 4096 nA range at 7.2 mA LEDs, a placed finger
     * typically produces IR DC values well above 50 000.  Values near zero
     * indicate no contact.
     */
    float ir_mean  = mean_u32(s_ir,  N_SAMPLES);
    float red_mean = mean_u32(s_red, N_SAMPLES);
    if (ir_mean < 50000.0f) {
        s_n = 0;   /* reset so we get a clean window when finger returns */
        return ESP_OK;
    }

    /* ── 6. Heart rate ── */
    /*
     * Effective sample rate = 25 samples/s (100 SPS / 4× averaging).
     * Window duration = N_SAMPLES / 25.0 s = 4 s.
     * BPM = peaks_in_window × (60 / window_duration_s).
     */
    int   peaks    = count_peaks(s_ir, N_SAMPLES, ir_mean);
    float window_s = N_SAMPLES / 25.0f;
    float bpm      = (float)peaks * (60.0f / window_s);

    if (bpm >= 40.0f && bpm <= 220.0f) {
        out->heart_rate_bpm = bpm;
        out->hr_valid       = true;
    }

    /* ── 7. SpO2 ── */
    /*
     * R = (AC_red / DC_red) / (AC_ir / DC_ir)
     * SpO2 ≈ 110 – 25 × R   (empirical linear approximation)
     *
     * AC is estimated as the peak-to-peak amplitude of each channel.
     */
    uint32_t ir_max  = s_ir [0], ir_min  = s_ir [0];
    uint32_t red_max = s_red[0], red_min = s_red[0];
    for (int i = 1; i < N_SAMPLES; i++) {
        if (s_ir [i] > ir_max ) ir_max  = s_ir [i];
        if (s_ir [i] < ir_min ) ir_min  = s_ir [i];
        if (s_red[i] > red_max) red_max = s_red[i];
        if (s_red[i] < red_min) red_min = s_red[i];
    }

    float ac_ir  = (float)(ir_max  - ir_min );
    float ac_red = (float)(red_max - red_min);

    if (ac_ir > 0.0f && ir_mean > 0.0f && red_mean > 0.0f) {
        float R    = (ac_red / red_mean) / (ac_ir / ir_mean);
        float spo2 = 110.0f - 25.0f * R;
        if (spo2 >= 70.0f && spo2 <= 100.0f) {
            out->spo2_pct   = spo2;
            out->spo2_valid = true;
        }
    }

    ESP_LOGD(TAG, "HR=%.1f BPM (valid=%d)  SpO2=%.1f%% (valid=%d)",
             out->heart_rate_bpm, out->hr_valid,
             out->spo2_pct, out->spo2_valid);

    return ESP_OK;
}
