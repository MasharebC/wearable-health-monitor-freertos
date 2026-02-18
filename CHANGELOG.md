# Changelog

## 2026-02-18
- Renamed project directory to remove whitespace (`Health-Monitor-Device-2`) for PlatformIO compatibility.
- Fixed flash-size mismatch by setting `board_upload.flash_size = 2MB` in `platformio.ini`.
- Refactored and stabilized MLX90614 + SSD1306 firmware.
- Added fixed-rate 1 second refresh using `vTaskDelayUntil`.
- Added temperature smoothing (5-sample moving average).
- Added unit toggle (C/F) using BOOT button (`GPIO0`).
- Added fever alert (`ALERT: FEVER`) based on smoothed object temp threshold.
- Added and validated unit tests for conversion math (`test/test_temp_math`).
