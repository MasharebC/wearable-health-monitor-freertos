#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include <stdint.h>

/*
 * Freeze these values once you complete timing validation.
 * Keep changes localized to this file for interview traceability.
 */

/* Feature toggles (1 = enabled, 0 = disabled) */
#define APP_FEATURE_TEMP_SENSOR      1U
#define APP_FEATURE_PPG_SENSOR       0U
#define APP_FEATURE_IMU_SENSOR       0U
#define APP_FEATURE_BLE_TELEMETRY    0U
#define APP_FEATURE_FAULT_INJECTION  1U

/* Sampling periods (ms) */
#define APP_PERIOD_TEMP_MS           1000U
#define APP_PERIOD_PPG_MS            10U
#define APP_PERIOD_IMU_MS            20U
#define APP_PERIOD_DIAG_MS           1000U

/* Queue configuration */
#define APP_Q_SENSOR_RAW_LEN         16U
#define APP_Q_PACKET_OUT_LEN         16U

/* Task stack sizes (words) */
#define APP_STACK_SENSOR_ACQ_WORDS   4096U
#define APP_STACK_PROCESSING_WORDS   4096U
#define APP_STACK_TELEMETRY_WORDS    4096U
#define APP_STACK_DIAG_WORDS         3072U

/* Task priorities (higher value => higher priority) */
#define APP_PRIO_SENSOR_ACQ          4U
#define APP_PRIO_PROCESSING          3U
#define APP_PRIO_TELEMETRY           2U
#define APP_PRIO_DIAG                1U

/* Packet metadata */
#define APP_PACKET_VERSION           1U

/* Sensor sanity limits */
#define APP_TEMP_C_MIN               (-40.0f)
#define APP_TEMP_C_MAX               (125.0f)

/* Fault injection controls */
#define APP_FAULT_NONE               0U
#define APP_FAULT_I2C_CONTENTION     1U
#define APP_FAULT_TX_SLOWDOWN        2U

#endif /* APP_CONFIG_H_ */
