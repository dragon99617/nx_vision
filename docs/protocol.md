# Gimbal Protocol

Runtime packet:

```text
A,yaw_cdeg,pitch_cdeg,dist_mm,valid\n
```

- `yaw_cdeg`: relative yaw angle error, `0.01 deg`.
- `pitch_cdeg`: relative pitch angle error, `0.01 deg`.
- `dist_mm`: laser-origin-to-target distance estimate.
- `valid`: `1` for valid target, `0` for no target.

The STM32 side should compute:

```text
yaw_target = current_yaw + yaw_delta
pitch_target = current_pitch + pitch_delta
```

This replaces the old pixel PID outer loop.

