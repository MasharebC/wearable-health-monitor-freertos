# ESP32 Health Monitor Firmware (FreeRTOS Portfolio Project)

## Project Summary
This project is an interview-focused embedded firmware portfolio build for ESP32.
The goal is to demonstrate production-style RTOS architecture, reliability, and
debugging discipline using an existing health-sensor codebase, not feature count.

## Scope (Frozen)
- Keep existing hardware and sensors only.
- Implement deterministic periodic acquisition and telemetry pipeline.
- Use explicit FreeRTOS ownership boundaries and inter-task messaging.
- Add diagnostics and fault-injection for debug storytelling.
- Document measurable stability criteria and pass/fail outcomes.

## System Architecture
### Core tasks
- `task_sensor_acq` (high priority): periodic reads, I2C ownership, raw frame output.
- `task_processing` (medium priority): sanity checks/filters, packet assembly.
- `task_telemetry` (medium-low priority): UART/BLE output, backpressure counters.
- `task_diag` (low priority): stack/heap/queue/error telemetry at 1 Hz.

### IPC and synchronization
- Queue: `q_sensor_raw` (`sensor_acq` -> `processing`)
- Queue: `q_packet_out` (`processing` -> `telemetry`)
- Mutex: `i2c_mutex` (all sensor bus transactions)
- Event group or status bits: runtime health/fault flags

See `docs/architecture.md` for task responsibilities, priorities, and packet flow.

## Build, Flash, Monitor
```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

## Runtime Behavior
- Fixed-rate sensor sampling using `vTaskDelayUntil`.
- Timestamped output packets with versioned schema + status flags.
- Runtime diagnostics:
- Task stack high-water marks
- Queue depth/high-water counters
- I2C transaction errors/timeouts
- Telemetry drops and processing latency

## Reliability and Debugging
Two intentional failures are used for reproducible debugging:
- I2C contention/timeout: demonstrates mutex discipline and bounded retry logic.
- Telemetry slowdown causing queue pressure: demonstrates backpressure handling.

Expected debug artifacts:
- Counter deltas before/after fix
- Root cause explanation
- Post-fix stability run results

## Validation Strategy
- Enable assertions (`configASSERT`) for invariants and API assumptions.
- Add sanity checks on sensor ranges and timestamp monotonicity.
- Add host-side packet parser tests in `test/` (header/length/CRC/timestamp checks).
- Run soak tests (target: >=30 minutes) and record crash/reset behavior.

## Hardware and Wiring
Current hardware integration in this repo includes:
- ESP32 (`freenove_esp32_wrover`)
- MLX90614
- SSD1306 I2C OLED
- Status LED

Keep wiring and board details aligned with your current implementation.

## Interview Talking Points
- Why queue-based ownership was chosen over shared globals.
- How task priorities were selected by latency sensitivity.
- How failures were intentionally reproduced and fixed.
- What metrics were used to prove deterministic behavior.
- Which features were intentionally not added to protect schedule and quality.

## Non-Goals
- No new sensors unless required to restore existing functionality.
- No cloud/mobile app expansion.
- No medical-grade claims.
- No major feature expansion after architecture freeze.

## Done Criteria (Application Ready)
- Stable runtime with no crash/reset during soak test.
- Measured timing, queue, stack, and error metrics are documented.
- At least one injected failure is reproducibly debugged and fixed.
- README + architecture doc are clear enough to explain in <5 minutes.
