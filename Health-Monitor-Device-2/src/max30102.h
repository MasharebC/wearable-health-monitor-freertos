#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#define MAX30102_I2C_ADDR 0x57

typedef struct {
    float heart_rate_bpm;   /* beats per minute; check hr_valid  */
    float spo2_pct;         /* 0–100 %; check spo2_valid         */
    bool  hr_valid;
    bool  spo2_valid;
} max30102_result_t;

/**
 * Verify the MAX30102 is on the bus and configure it for SpO2 + HR mode.
 * Call once after the shared I2C bus is initialised.
 */
esp_err_t max30102_init(i2c_port_t port);

/**
 * Drain the FIFO and, once the internal rolling buffer is full, compute
 * heart rate and SpO2.  Call every ~1 s from your sensor task.
 *
 * hr_valid / spo2_valid are false until enough data has been collected
 * and the signal looks plausible (finger present, values in range).
 */
esp_err_t max30102_read_vitals(i2c_port_t port, max30102_result_t *out);
