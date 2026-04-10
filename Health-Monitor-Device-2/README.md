# Health Monitor Device 2

ESP32 firmware (PlatformIO + ESP-IDF 5.5) that reads three vital signs and
streams them over BLE to a phone every second.

## Hardware

| Component | Interface | Address |
|---|---|---|
| ESP32-DevKit | — | — |
| HiLetgo GY-906 MLX90614ESF (IR thermometer) | I2C SDA=21 SCL=22 | 0x5A |
| MAX30102 (heart rate + SpO2) | I2C SDA=21 SCL=22 | 0x57 |
| SSD1306 OLED 128×64 (optional) | I2C SDA=21 SCL=22 | 0x3C |

I2C bus runs at 100 kHz — required by the MLX90614 (SMBus spec, max 100 kHz).

## BLE Payload

Advertises as `HealthMonitor`. One custom GATT service, one notify+read
characteristic. New reading pushed every second:

```json
{"temp_c":36.4,"hr_bpm":72.0,"hr_valid":true,"spo2_pct":97.1,"spo2_valid":true}
```

`hr_valid` and `spo2_valid` are `false` for the first ~4 s while the MAX30102
rolling buffer fills, and whenever no finger is detected on the sensor.

## Architecture

Three FreeRTOS tasks with explicit ownership boundaries:

```
sensor_task (priority 5)
    MLX90614 + MAX30102 reads every 1 s
    xQueueOverwrite → ble_queue, display_queue
    Hardware watchdog registered (10 s timeout)

ble_task (priority 4)
    xQueueReceive ← ble_queue
    Format JSON → update latest_payload under mutex → BLE notify

display_task (priority 3)
    xQueueReceive ← display_queue
    Redraw OLED only on value change
    Drains silently if display not connected
```

## Source Files

| File | Purpose |
|---|---|
| `src/main.c` | FreeRTOS tasks, BLE GATT server, I2C init |
| `src/mlx90614.c/.h` | MLX90614 SMBus driver |
| `src/max30102.c/.h` | MAX30102 driver + HR/SpO2 algorithm |
| `src/ssd1306.c/.h` | SSD1306 frame-buffer display driver |

## Phone Testing

Use nRF Connect or LightBlue. Connect to `HealthMonitor`, find the custom
service (UUID starts with `C0FFEE00`), and enable notifications on the
vitals characteristic.
