# FreeRTOS Architecture

## Goals
- Deterministic periodic sampling
- Clear inter-task ownership and interfaces
- Reproducible failure/debug workflow
- Lightweight runtime observability

## Task Model
1. `task_sensor_acq` (priority 4)
- Owns periodic read schedule with `vTaskDelayUntil`.
- Reads active sensors over I2C under `i2c_mutex`.
- Writes `sensor_frame_t` to `q_sensor_raw`.
- Updates sensor error/timeout counters.

2. `task_processing` (priority 3)
- Blocks on `q_sensor_raw`.
- Performs sanity checks, smoothing/filtering, and status-bit updates.
- Writes `health_packet_t` to `q_packet_out`.
- Tracks processing duration and dropped-frame count.

3. `task_telemetry` (priority 2)
- Blocks on `q_packet_out`.
- Encodes and transmits packet over UART (BLE optional if already stable).
- Tracks tx latency and queue-pressure drops.

4. `task_diag` (priority 1, periodic 1 Hz)
- Prints/report counters and watermarks.
- Reads queue high-water, stack high-water, and heap minimum.
- Emits fault flags for stale sensor data or persistent tx drops.

## Inter-Task Communication
- Queue: `q_sensor_raw` (acq -> processing)
- Queue: `q_packet_out` (processing -> telemetry)
- Mutex: `i2c_mutex` (all I2C peripheral access)
- Event group/status bits: health + fault indicators

No task directly mutates another task's working state. Shared mutable data is
limited to counters protected by atomic operations or short critical sections.

## Timing Budget (Default Targets)
- Temperature sample period: 1000 ms
- PPG sample period: 10 ms (if enabled)
- IMU sample period: 20 ms (if enabled)
- Diagnostics report period: 1000 ms

Adjust rates only in `include/app_config.h` and keep them frozen once validated.

## Intentional Fault Scenarios
1. I2C contention timeout
- Fault: bypass mutex path or inject extra bus-hold delay.
- Signals: timeout counter rise, stale sample flags, queue under-runs.
- Fix: enforce single bus path + bounded retry/backoff.

2. Telemetry backpressure
- Fault: inject artificial tx delay.
- Signals: `q_packet_out` fill level increases, drop counter rises.
- Fix: bounded queue policy + non-blocking/shorter tx path.

## Validation Checklist
- `configASSERT` enabled in debug builds
- Packet format checks (version/length/CRC/timestamp monotonicity)
- Queue high-water below threshold under nominal load
- Stack high-water margin observed for every task
- >=30 min soak with zero resets and stable counters
