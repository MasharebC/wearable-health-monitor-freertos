#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
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

#include "mlx90614.h"
#include "ssd1306.h"

#define DEVICE_NAME      "HealthMonitor"
#define STREAM_PERIOD_MS 1000

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
#define I2C_PORT     I2C_NUM_0
#define I2C_FREQ_HZ  100000  /* MLX90614 is SMBus — max 100 kHz */

static const char *TAG = "health_ble";

static uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t vitals_char_handle;
static uint8_t own_addr_type;

static bool display_ok = false;

static char latest_payload[128] = "{\"status\":\"booting\"}";

static const ble_uuid128_t health_service_uuid =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90,
                     0x78, 0x56, 0x34, 0x12, 0x00, 0xee, 0xff, 0xc0);

static const ble_uuid128_t vitals_char_uuid =
    BLE_UUID128_INIT(0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90,
                     0x78, 0x56, 0x34, 0x12, 0x01, 0xee, 0xff, 0xc0);

void ble_store_config_init(void);

static void ble_app_advertise(void);

/* ---- I2C helpers ---- */

static esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
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

/* ---- BLE callbacks ---- */

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE stack reset, reason=%d", reason);
}

static void notify_phone(void)
{
    if (ble_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(latest_payload,
                                                strlen(latest_payload));
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate notification buffer");
        return;
    }

    int rc = ble_gatts_notify_custom(ble_conn_handle, vitals_char_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "Notify failed: %d", rc);
    }
}

static int vitals_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    int rc = os_mbuf_append(ctxt->om, latest_payload, strlen(latest_payload));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &health_service_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]) {
                {
                    .uuid = &vitals_char_uuid.u,
                    .access_cb = vitals_access_cb,
                    .val_handle = &vitals_char_handle,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
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
        ESP_LOGI(TAG, "Advertising complete, restarting");
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
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields scan_rsp_fields;
    memset(&scan_rsp_fields, 0, sizeof(scan_rsp_fields));
    scan_rsp_fields.uuids128 = (ble_uuid128_t *)&health_service_uuid;
    scan_rsp_fields.num_uuids128 = 1;
    scan_rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&scan_rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising as %s", DEVICE_NAME);
    }
}

static void ble_on_sync(void)
{
    uint8_t addr_val[6] = {0};
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) { ESP_LOGE(TAG, "ensure_addr failed: %d", rc); return; }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) { ESP_LOGE(TAG, "infer_auto failed: %d", rc); return; }

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

/* ---- Sensor + display + BLE streaming task ---- */

static void sensor_stream_task(void *param)
{
    (void)param;
    char line[32];

    /* Track last values to avoid redundant display redraws. */
    float last_temp_c = -999.0f;
    bool last_connected = false;

    while (true) {
        float temp_c = 0.0f;
        esp_err_t ret = mlx90614_read_object_temp(I2C_PORT, &temp_c);

        if (ret == ESP_OK) {
            snprintf(latest_payload, sizeof(latest_payload),
                     "{\"temperature_c\":%.2f}", temp_c);

            bool connected = (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE);
            bool temp_changed = (temp_c != last_temp_c);
            bool conn_changed = (connected != last_connected);

            if (display_ok && (temp_changed || conn_changed)) {
                ssd1306_clear();
                ssd1306_draw_string(0, 0, "Health Monitor");
                snprintf(line, sizeof(line), "Temp: %.2f C", temp_c);
                ssd1306_draw_string(0, 16, line);
                ssd1306_draw_string(0, 40, connected ? "BLE: Connected"
                                                     : "BLE: Advertising");
                ssd1306_update(I2C_PORT);
            }

            last_temp_c = temp_c;
            last_connected = connected;

            ESP_LOGI(TAG, "Temp=%.2f C  payload=%s", temp_c, latest_payload);
            notify_phone();
        } else {
            ESP_LOGW(TAG, "Temp read failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(STREAM_PERIOD_MS));
    }
}

/* ---- Entry point ---- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(i2c_bus_init());
    ESP_LOGI(TAG, "I2C bus initialized");

    int found = 0;
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=%d SCL=%d)...", I2C_SDA_PIN, I2C_SCL_PIN);
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_probe_device(I2C_PORT, addr) == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "Scan done, %d device(s) found", found);

    bool temp_sensor_ok = (mlx90614_init(I2C_PORT) == ESP_OK);
    if (!temp_sensor_ok) {
        ESP_LOGW(TAG, "MLX90614 not found — continuing without temp sensor");
    }

    display_ok = (ssd1306_init(I2C_PORT) == ESP_OK);
    if (!display_ok) {
        ESP_LOGW(TAG, "SSD1306 not found — continuing without display");
        i2c_bus_recover(I2C_PORT);
    }

    ESP_LOGI(TAG, "Temp sensor: %s, Display: %s",
             temp_sensor_ok ? "OK" : "FAIL",
             display_ok ? "OK" : "FAIL");

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    ESP_ERROR_CHECK(rc);
    rc = ble_gatts_add_svcs(gatt_svcs);
    ESP_ERROR_CHECK(rc);

    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(DEVICE_NAME));
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(sensor_stream_task, "sensor_stream", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "App started");
}
