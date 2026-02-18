# Wearable Health Monitor (FreeRTOS + BLE, ESP32)

## Overview
A wearable health monitoring device targeting real-time vital sensing on ESP32.

Current implemented firmware in this repo revision focuses on:
- MLX90614 temperature sensing (ambient + object)
- SSD1306 OLED display
- 1-second fixed refresh
- C/F unit toggle via BOOT button
- Moving-average smoothing
- Fever alert thresholding

Planned system scope includes PPG, IMU fall detection, BLE telemetry, and multi-task FreeRTOS architecture.

## Current Implemented Features
- Reads ambient/object temperature from MLX90614 over I2C.
- Displays values on SSD1306 OLED.
- Updates OLED at a fixed 1-second interval.
- 5-sample moving-average smoothing for stable readings.
- Celsius/Fahrenheit display toggle using ESP32 BOOT button (`GPIO0`).
- Fever alert when smoothed object temperature is `>= 38.0 C`.

## Hardware (Current)
- ESP32 (`freenove_esp32_wrover`)
- MLX90614 temperature sensor
- SSD1306 I2C OLED (`0x3C`)

## Wiring (Current)
- ESP32 `GPIO21` -> I2C `SDA` (MLX90614 + OLED SDA)
- ESP32 `GPIO22` -> I2C `SCL` (MLX90614 + OLED SCL)
- ESP32 `3V3` -> sensor and OLED VCC
- ESP32 `GND` -> sensor and OLED GND
- ESP32 `GPIO0` (BOOT button) -> toggles `C/F`

## Build, Flash, Monitor (PlatformIO)
```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

## OLED UI
- Line 1: sensor title + active unit (`C`/`F`)
- Line 3: ambient temperature (smoothed)
- Line 5: object temperature (smoothed)
- Line 7: status (`Normal` or `ALERT: FEVER`)
- Line 8: button hint (`BTN0: C/F Toggle`)

## Planned Expansion
- MAX30102 PPG for HR/SpO2
- MPU-6050 fall detection
- BLE GATT live telemetry
- FreeRTOS task decomposition for sensing/signal/UI/BLE/alerts
- Data logging and battery optimization

## Notes
- Fever threshold is configured in `src/main.c` via `FEVER_THRESHOLD_C`.
- If sensor reads fail, OLED shows `Read Fail`; verify wiring/address.
