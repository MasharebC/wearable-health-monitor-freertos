#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "os/os_mbuf.h"

#include "max30102.h"
#include "mlx90614.h"
#include "ssd1306.h"

/* ── Build-time config ──────────────────────────────────────────────── */

#define DEVICE_NAME       "HealthMonitor"
#define STREAM_PERIOD_MS  1000
#define SENSOR_WDT_MS     10000   /* reboot if sensor_task hangs > 10 s */

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
#define I2C_PORT     I2C_NUM_0
#define I2C_FREQ_HZ  100000       /* MLX90614 is SMBus — max 100 kHz    */

static const char *TAG = "health_ble";

/* ── Vitals data type passed between tasks ──────────────────────────── */

typedef struct {
    float temp_c;
    bool  temp_valid;
    float heart_rate_bpm;
    bool  hr_valid;
    float spo2_pct;
    bool  spo2_valid;
} vitals_t;

/* ── Inter-task communication ───────────────────────────────────────── */
/*
 * Depth-1 "mailbox" queues — xQueueOverwrite ensures the consumer always
 * sees the most recent reading; no reading is ever blocked waiting for
 * stale data to drain.
 */
static QueueHandle_t    ble_queue;     /* sensor_task → ble_task     */
static QueueHandle_t    display_queue; /* sensor_task → display_task */

/*
 * Mutex protecting latest_payload.
 * Written by ble_task; read by vitals_access_cb (NimBLE stack context).
 */
static SemaphoreHandle_t payload_mutex;

/* ── Shared BLE state ───────────────────────────────────────────────── */

static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t vitals_char_handle;
static uint8_t  own_addr_type;
static char     latest_payload[128] = "{\"status\":\"booting\"}";

/* ── BLE service / characteristic UUIDs ────────────────────────────── */

static const ble_uuid128_t health_service_uuid =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90,
                     0x78, 0x56, 0x34, 0x12, 0x00, 0xee, 0xff, 0xc0);

static const ble_uuid128_t vitals_char_uuid =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90,
                     0x78, 0x56, 0x34, 0x12, 0x01, 0xee, 0xff, 0xc0);

void ble_store_config_init(void);
static void ble_app_advertise(void);

/* ── I2C helpers ────────────────────────────────────────────────────── */

static esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = I2C_SDA_PIN,
        .scl_io_num    = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_PORT, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/* Probe a single 7-bit I2C address — returns ESP_OK if ACK received. */
static esp_err_t i2c_probe_device(i2c_port_t port, uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*
 * Reset the I2C peripheral after a failed transaction.
 * A NACK from an absent slave can leave the ESP32 I2C hardware in a fault
 * state where subsequent transactions also fail.
 */
static void i2c_bus_recover(i2c_port_t port)
{
    i2c_driver_delete(port);
    ESP_ERROR_CHECK(i2c_bus_init());
}

/* ── BLE helpers ────────────────────────────────────────────────────── */

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset, reason=%d", reason);
}

/*
 * Send a BLE notification with an explicit payload buffer.
 * Called from ble_task — does not touch latest_payload.
 */
static void notify_phone(const char *payload, size_t len)
{
    if (ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate notification buffer");
        return;
    }
    int rc = ble_gatts_notify_custom(ble_conn_handle, vitals_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed: %d", rc);
    }
}

/*
 * GATT read callback — called from NimBLE stack context.
 * Acquires payload_mutex so it cannot race with ble_task's write.
 */
static int vitals_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    xSemaphoreTake(payload_mutex, portMAX_DELAY);
    int rc = os_mbuf_append(ctxt->om, latest_payload, strlen(latest_payload));
    xSemaphoreGive(payload_mutex);

    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &health_service_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]) {
                {
                    .uuid       = &vitals_char_uuid.u,
                    .access_cb  = vitals_access_cb,
                    .val_handle = &vitals_char_handle,
                    .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                },
                {0},
            },
    },
    {0},
};

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ble_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Phone connected");
        } else {
            ESP_LOGW(TAG, "Connect failed, restarting advertising");
            ble_app_advertise();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "Phone disconnected");
        ble_app_advertise();
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Client subscribe: attr=%u notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_app_advertise();
        return 0;
    default:
        return 0;
    }
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags               = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl          = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name                = (uint8_t *)DEVICE_NAME;
    fields.name_len            = strlen(DEVICE_NAME);
    fields.name_is_complete    = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields: %d", rc); return; }

    struct ble_hs_adv_fields scan_rsp;
    memset(&scan_rsp, 0, sizeof(scan_rsp));
    scan_rsp.uuids128             = (ble_uuid128_t *)&health_service_uuid;
    scan_rsp.num_uuids128         = 1;
    scan_rsp.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp);
    if (rc != 0) { ESP_LOGE(TAG, "adv_rsp_set_fields: %d", rc); return; }

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", DEVICE_NAME);
    }
}

static void ble_on_sync(void)
{
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) { ESP_LOGE(TAG, "ensure_addr: %d", rc); return; }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "infer_auto: %d", rc); return; }

    ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "BLE ready, addr: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0]);
    ble_app_advertise();
}

static void ble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ═══════════════════════════════════════════════════════════════════════
 * sensor_task  (priority 5)
 *
 * Owns all I2C sensor access. Pushes vitals_t onto both downstream queues
 * every STREAM_PERIOD_MS. Registered with the hardware task watchdog —
 * the device reboots automatically if this task hangs for SENSOR_WDT_MS.
 * ═══════════════════════════════════════════════════════════════════════ */
static void sensor_task(void *param)
{
    (void)param;

    bool temp_sensor_ok = (mlx90614_init(I2C_PORT) == ESP_OK);
    bool hr_sensor_ok   = (max30102_init(I2C_PORT)  == ESP_OK);

    ESP_LOGI(TAG, "sensor_task: temp=%s hr=%s",
             temp_sensor_ok ? "OK" : "FAIL",
             hr_sensor_ok   ? "OK" : "FAIL");

    /* Register this task with the hardware watchdog. */
    esp_task_wdt_add(NULL);

    while (true) {
        vitals_t v = {0};

        /* ── Temperature ── */
        if (temp_sensor_ok) {
            esp_err_t ret = mlx90614_read_object_temp(I2C_PORT, &v.temp_c);
            if (ret == ESP_OK) {
                v.temp_valid = true;
            } else {
                ESP_LOGW(TAG, "Temp read failed (%s) — recovering",
                         esp_err_to_name(ret));
                i2c_bus_recover(I2C_PORT);
                temp_sensor_ok = (mlx90614_init(I2C_PORT) == ESP_OK);
            }
        }

        /* ── Heart rate + SpO2 ── */
        if (hr_sensor_ok) {
            max30102_result_t hr = {0};
            esp_err_t ret = max30102_read_vitals(I2C_PORT, &hr);
            if (ret == ESP_OK) {
                v.heart_rate_bpm = hr.heart_rate_bpm;
                v.hr_valid       = hr.hr_valid;
                v.spo2_pct       = hr.spo2_pct;
                v.spo2_valid     = hr.spo2_valid;
            } else {
                ESP_LOGW(TAG, "HR read failed (%s) — recovering",
                         esp_err_to_name(ret));
                i2c_bus_recover(I2C_PORT);
                hr_sensor_ok = (max30102_init(I2C_PORT) == ESP_OK);
            }
        }

        /* Publish to consumers — overwrite so they always see latest data. */
        xQueueOverwrite(ble_queue,     &v);
        xQueueOverwrite(display_queue, &v);

        /* Feed the watchdog — proves this task is still alive. */
        esp_task_wdt_reset();

        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * ble_task  (priority 4)
 *
 * Receives vitals from sensor_task, formats the JSON payload, updates the
 * shared latest_payload buffer under a mutex (so the GATT read callback
 * cannot race with this write), then pushes a BLE notification.
 * ═══════════════════════════════════════════════════════════════════════ */
static void ble_task(void *param)
{
    (void)param;
    vitals_t v;

    while (true) {
        /* Block until sensor_task posts a new reading. */
        xQueueReceive(ble_queue, &v, portMAX_DELAY);

        /* Format JSON into a local buffer — no lock needed here. */
        char buf[128];
        if (v.temp_valid) {
            snprintf(buf, sizeof(buf),
                     "{\"temp_c\":%.2f"
                     ",\"hr_bpm\":%.1f,\"hr_valid\":%s"
                     ",\"spo2_pct\":%.1f,\"spo2_valid\":%s}",
                     v.temp_c,
                     v.heart_rate_bpm, v.hr_valid   ? "true" : "false",
                     v.spo2_pct,       v.spo2_valid  ? "true" : "false");
        } else {
            snprintf(buf, sizeof(buf),
                     "{\"temp_c\":null"
                     ",\"hr_bpm\":%.1f,\"hr_valid\":%s"
                     ",\"spo2_pct\":%.1f,\"spo2_valid\":%s}",
                     v.heart_rate_bpm, v.hr_valid   ? "true" : "false",
                     v.spo2_pct,       v.spo2_valid  ? "true" : "false");
        }

        /*
         * Update the shared payload under mutex.
         * vitals_access_cb (NimBLE context) reads latest_payload — the mutex
         * prevents a torn read if a GATT client polls exactly now.
         */
        xSemaphoreTake(payload_mutex, portMAX_DELAY);
        memcpy(latest_payload, buf, sizeof(latest_payload));
        xSemaphoreGive(payload_mutex);

        ESP_LOGI(TAG, "payload=%s", buf);

        /* Notify with the local buffer — no need to re-lock. */
        notify_phone(buf, strlen(buf));
    }
}

/* ═══════════════════════════════════════════════════════════════════════
 * display_task  (priority 3)
 *
 * Receives vitals from sensor_task and redraws the OLED only when the
 * displayed values change.  If the display was not found at boot the task
 * simply drains the queue and discards, keeping the queue from filling.
 * ═══════════════════════════════════════════════════════════════════════ */
static void display_task(void *param)
{
    /* display_ok is passed as a cast intptr_t through pvParameters. */
    bool display_ok = (bool)(intptr_t)param;

    char line[32];
    vitals_t v;
    float last_temp        = -999.0f;
    bool  last_connected   = false;
    bool  last_hr_valid    = false;
    bool  last_spo2_valid  = false;

    while (true) {
        xQueueReceive(display_queue, &v, portMAX_DELAY);

        if (!display_ok) continue;  /* drain queue silently if no display */

        bool connected = (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE);

        bool changed = (v.temp_c      != last_temp)       ||
                       (connected     != last_connected)   ||
                       (v.hr_valid    != last_hr_valid)    ||
                       (v.spo2_valid  != last_spo2_valid);

        if (!changed) continue;

        ssd1306_clear();
        ssd1306_draw_string(0, 0, "Health Monitor");

        if (v.temp_valid) {
            snprintf(line, sizeof(line), "Temp: %.2f C", v.temp_c);
            ssd1306_draw_string(0, 12, line);
        }
        if (v.hr_valid) {
            snprintf(line, sizeof(line), "HR:   %.0f bpm", v.heart_rate_bpm);
            ssd1306_draw_string(0, 24, line);
        }
        if (v.spo2_valid) {
            snprintf(line, sizeof(line), "SpO2: %.1f%%", v.spo2_pct);
            ssd1306_draw_string(0, 36, line);
        }
        ssd1306_draw_string(0, 52, connected ? "BLE: Connected"
                                             : "BLE: Advertising");
        ssd1306_update(I2C_PORT);

        last_temp       = v.temp_c;
        last_connected  = connected;
        last_hr_valid   = v.hr_valid;
        last_spo2_valid = v.spo2_valid;
    }
}

/* ── Entry point ────────────────────────────────────────────────────── */

void app_main(void)
{
    /* NVS required by BLE stack. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_LOGI(TAG, "I2C bus initialised (SDA=%d SCL=%d @ %d Hz)",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    /* Scan — useful during development to confirm what's on the bus. */
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_probe_device(I2C_PORT, addr) == ESP_OK) {
            ESP_LOGI(TAG, "  I2C device at 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan: %d device(s)", found);

    /*
     * Initialise each peripheral.  If a peripheral is absent its init fails
     * gracefully; i2c_bus_recover() clears any fault state left by the
     * failed transaction so the next device can still be reached.
     */
    bool display_ok = (ssd1306_init(I2C_PORT) == ESP_OK);
    if (!display_ok) {
        ESP_LOGW(TAG, "SSD1306 not found");
        i2c_bus_recover(I2C_PORT);
    }

    /* Create inter-task primitives before starting any task. */
    ble_queue     = xQueueCreate(1, sizeof(vitals_t));
    display_queue = xQueueCreate(1, sizeof(vitals_t));
    payload_mutex = xSemaphoreCreateMutex();
    configASSERT(ble_queue && display_queue && payload_mutex);

    /* Configure hardware task watchdog. */
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms    = SENSOR_WDT_MS,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    /* BLE stack init. */
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK(rc);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK(rc);

    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(DEVICE_NAME));
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);

    /*
     * Task priorities:
     *   5 — sensor_task   : drives the pipeline; highest so reads are timely
     *   4 — ble_task      : formats and sends; lower so sensor is never starved
     *   3 — display_task  : lowest; display lag is acceptable
     */
    xTaskCreate(sensor_task,  "sensor",  4096, NULL,                    5, NULL);
    xTaskCreate(ble_task,     "ble_out", 4096, NULL,                    4, NULL);
    xTaskCreate(display_task, "display", 2048, (void*)(intptr_t)display_ok, 3, NULL);

    ESP_LOGI(TAG, "App started — tasks running");
}
