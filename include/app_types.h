#ifndef APP_TYPES_H_
#define APP_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SENSOR_SRC_TEMP = 0,
    SENSOR_SRC_PPG = 1,
    SENSOR_SRC_IMU = 2
} sensor_source_t;

typedef enum {
    APP_STATUS_OK                  = 0U,
    APP_STATUS_TEMP_INVALID        = (1U << 0),
    APP_STATUS_I2C_ERROR           = (1U << 1),
    APP_STATUS_QUEUE_DROP          = (1U << 2),
    APP_STATUS_STALE_SENSOR_DATA   = (1U << 3),
    APP_STATUS_TX_BACKPRESSURE     = (1U << 4),
    APP_STATUS_FAULT_INJECT_ACTIVE = (1U << 5)
} app_status_bits_t;

typedef struct {
    uint32_t timestamp_ms;
    sensor_source_t source;
    float temp_c;
    float temp_ambient_c;
    bool valid;
} sensor_frame_t;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t version;
    uint16_t sequence;
    uint32_t status_bits;
    float temp_c;
    float temp_ambient_c;
    uint16_t crc16;
} health_packet_t;

typedef struct {
    uint32_t i2c_errors;
    uint32_t sensor_read_errors;
    uint32_t raw_queue_drops;
    uint32_t packet_queue_drops;
    uint32_t tx_drops;
    uint32_t stale_sensor_events;
    uint32_t assertions_hit;
} app_diag_counters_t;

#endif /* APP_TYPES_H_ */
