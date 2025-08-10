# Wearable Health Monitor (FreeRTOS + BLE, ESP32)

## Overview
A **wearable health monitoring device** that continuously measures:
- Heart Rate
- SpO₂ (Blood Oxygen)
- Body Temperature
- Fall Detection

Streams data via **Bluetooth Low Energy (BLE)** and runs on **FreeRTOS** for real-time task management.  
Phase 2 adds **ECG** and **Blood Pressure estimation** using Pulse Transit Time (PTT).

---

## Why This Matters
- Continuous vitals monitoring outside hospitals is valuable for elderly care, post-surgery recovery, and chronic illness management.
- Combines **IoT**, **embedded firmware**, **health sensors**, and **BLE**, all in-demand skills for firmware/embedded engineering roles.
- Demonstrates ability to design, implement, and document a complete hardware + firmware system.

---

## Phase 1 Features (MVP)
- **MAX30102 PPG** → Heart Rate & SpO₂
- **MLX90614 IR** or **DS18B20 contact** → Temperature
- **MPU-6050 IMU** → Fall detection
- **SSD1306 OLED** → Local vitals display
- **BLE GATT server** for live data
- **FreeRTOS** tasks, queues, and watchdog
- **Buzzer alerts** for emergencies

---

## Phase 2 Features (Planned)
- ECG via AD8232 or MAX30003
- Blood Pressure estimation (PTT from ECG↔PPG)
- SD card logging
- Battery-powered wearable enclosure
- Cloud dashboard for remote monitoring
- Optional STM32WB55 port for medical-grade BLE

---

## Hardware — Phase 1 Logistics
| Item | Purpose | Notes |
|------|---------|-------|
| ESP32-WROVER Dev Kit (Freenove kit) | Main MCU, BLE, FreeRTOS | Comes with breadboard, jumpers, buzzer |
| MAX30102 | Heart Rate & SpO₂ | I²C interface |
| MPU-6050 | Fall detection (accel+gyro) | I²C interface |
| MLX90614 or DS18B20 | Temperature | IR or contact-based |
| SSD1306 0.96" OLED | Display vitals locally | I²C |
| Buzzer | Alerts | Included in kit |
| Mac USB Port | Power + programming | No external PSU needed for Phase 1 |

---

## Wiring (I²C Shared)
- **3.3V** → All sensors VCC  
- **GND** → All sensors GND  
- **SDA/SCL** → MAX30102, MPU-6050, MLX90614/SSD1306  
- Pull-ups: 4.7kΩ (many breakout boards already have them)  

---

## BLE GATT (Draft)
- **Service:** Health Monitor (UUID TBD)
  - `VITALS_LIVE` (notify): `{hr_bpm:uint16, spo2:uint8, temp_c_q10:int16}`
  - `ALERT_FLAGS` (notify): Bitfield `{FALL, FEVER, TACHY, HYPOXIA}`
  - `CONFIG` (write): Set alert thresholds
  - `STATUS` (read): Uptime, firmware version

---

## Repository Layout
```
firmware/
  CMakeLists.txt
  sdkconfig.defaults
  main/
    main.c
    app_cfg.h
    app_types.h
    tasks_ppg.c
    tasks_imu.c
    tasks_sigproc.c
    tasks_alerts.c
    tasks_ble.c
    tasks_ui.c
  drivers/
    max30102.c/.h
    mpu6050.c/.h
    ssd1306.c/.h
    mlx90614.c/.h
  include/
    ble_gatt.h
docs/
  test-plan.md
  power-notes.md
  enclosure-notes.md
images/
tools/
  capture_log.py
```

---

## Milestones
- **Week 2:** Heart rate validated ±3–5 bpm vs pulse oximeter
- **Week 4:** Fall detection demo operational
- **Week 6:** BLE notifications streaming to smartphone
- **Week 8:** Final enclosure, README update, demo video

---

## Getting Started

### Prerequisites
- macOS (M1/M2 or Intel)
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
- Git
- USB cable for ESP32

### Clone the Repository
```bash
git clone https://github.com/<your-username>/wearable-health-monitor-freertos.git
cd wearable-health-monitor-freertos
```

### Build the Firmware
```bash
. $HOME/esp/esp-idf/export.sh
cd firmware
idf.py set-target esp32
idf.py build
```

### Flash & Monitor
```bash
idf.py -p /dev/tty.usb* flash monitor
```
Press `Ctrl+]` to exit.

---

## Future Work
- ML-based fall detection with TensorFlow Lite Micro
- Power optimization for longer battery life
- Cloud data pipeline for remote monitoring
- Clinical trials for accuracy validation

---

## License
MIT License — see `LICENSE`
eof
