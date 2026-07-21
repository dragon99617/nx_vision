# Gimbal protocol V4

`config/serial.yaml` now selects `protocol: v4`. V2 and `legacy_a` remain
explicit debug modes. After a V4 `HELLO/HELLO_ACK` exchange, the firmware locks
to binary mode until reset.

All integers are little-endian. Every frame is:

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | magic `0xA55A` |
| 2 | 1 | version `4` |
| 3 | 1 | message type |
| 4 | 2 | payload length |
| 6 | 4 | sequence |
| 10 | N | payload |
| 10+N | 4 | CRC32C over header and payload |

CRC32C uses the reflected Castagnoli polynomial `0x82F63B78`, initial value
`0xFFFFFFFF`, and final inversion.

## Messages

- `HELLO` (1) and `HELLO_ACK` (2): 4-byte capability mask.
- `SYNC_REQ` (3): host `steady_clock` transmit time in nanoseconds (`u64`).
- `SYNC_RESP` (4): echoed host time (`u64`), MCU receive TIM2 time (`u32 us`),
  and MCU transmit TIM2 time (`u32 us`).
- `CONTROL_SETPOINT` (5): 28-byte payload; total frame size 42 bytes.
- `ATTITUDE_STATE` (6): 50-byte payload; total frame size exactly 64 bytes.
- `FAULT_STATUS` (7): MCU time, fault bit mask, and packed diagnostic detail;
  it is emitted whenever IMU, calibration, CAN, or queue fault state changes.

`CONTROL_SETPOINT` payload order:

| Field | Wire unit |
|---|---|
| execution TIM2 time | `u32 us` |
| yaw position, yaw velocity | `i32 urad`, `i32 urad/s` |
| yaw torque feedforward | `i16 1e-4 N m` |
| pitch position, pitch velocity | `i32 urad`, `i32 urad/s` |
| pitch torque feedforward | `i16 1e-4 N m` |
| flags, reserved | `u16`, `u16` |

Control flags are valid, target-lost, velocity-feedforward-enabled, and
torque-feedforward-enabled in bits 0 through 3. Torque feedforward is disabled
in both default configurations.

`ATTITUDE_STATE` carries the actual IMU sample TIM2 time, body/camera-to-local-
world quaternion in Q30 (`w,x,y,z`), gyro in `mrad/s`, absolute yaw/pitch and
rates in `urad` units, last executed control sequence, IMU/control flags, and
the current 32-point queue depth.

The upper computer sends one planned point per millisecond with an adaptive
4--10 ms execution lead. The lower computer executes by TIM2 time. A single
missing 1 kHz point is interpolated. At 10 ms without an executable point it
holds position and clears both feedforwards; at 20 ms it holds the current
attitude and leaves visual tracking.

## Legacy debug formats

V2 remains `V2,seq,age_us,yaw_cdeg,pitch_cdeg,dist_mm,depth_age_ms,flags,crc16`.
`legacy_a` remains `A,yaw_cdeg,pitch_cdeg,dist_mm,valid`. Their relative-angle
world compensation remains available only for explicit debugging; production
world conversion is performed by `nx_vision`.
