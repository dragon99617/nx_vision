# yuntai Integration

The current `yuntai` project reads the first two CSV values as pixel errors and
then runs `pid_yaw_pixel` / `pid_pitch_pixel` to convert them to angle deltas.

For this project, replace that outer loop with direct relative angle input:

```text
old: dx_px,dy_px,0,0
new: A,yaw_cdeg,pitch_cdeg,dist_mm,valid
```

On valid frames:

```c
float yaw_delta_deg = yaw_cdeg * 0.01f;
float pitch_delta_rad = pitch_cdeg * 0.01f * DEG_TO_RAD;
yaw_control_target_deg = wrap_deg_0_360(BMI088_GetYawDeg() + yaw_delta_deg);
pitch_control_target_rad = pitch_clamp_target(pitch_motor.fb.pos + pitch_delta_rad);
```

On invalid or timeout frames, keep the current timeout behavior and reset visual
tracking state.

