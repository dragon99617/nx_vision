# Yuntai V4 integration

Production mode uses USB CDC V4 at 1 kHz. The firmware parser is fed directly
from the USB receive callback and replies with 64-byte attitude states from the
1 kHz control service. TIM2 remains the only execution and IMU timestamp.

The lower controller retains the position-rate cascade:

```text
rate_target = position_pid + velocity_feedforward
torque_command = rate_pid + torque_feedforward
```

Both intermediate rate commands and final torque commands are clamped by the
existing PID limits. Pitch soft limits and the existing 0.5 N m yaw / 1.0 N m
pitch safety limits remain authoritative. Signal-generator modes suppress V4
feedforward so identification excitation cannot accidentally combine with it.

Useful debugger state is under `g_transport_diag`, `g_gimbal_diag.yaw`, and
`g_gimbal_diag.pitch`. In particular inspect queue depth, CRC errors, execute
lateness, underflow count, velocity/torque feedforward, commanded torque, and
motor torque feedback. Run `node tools/update_ozone_watch.mjs` after regenerating
Ozone user files.

For dynamics identification, leave `torque_feedforward_enabled=0`, select only
one low-amplitude existing signal generator, and record a 1 kHz Ozone CSV. The
CSV columns required by `nx_vision/tools/fit_gimbal_feedforward.py` are `axis`,
`angular_velocity_rad_s`, `angular_acceleration_rad_s2`, `motor_torque_nm`, and
`position_rad`. Enable fitted torque feedforward only after held-out validation,
offline replay, and a hardware run that confirms all safety clamps.
