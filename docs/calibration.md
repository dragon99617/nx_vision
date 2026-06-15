# Calibration

Required before real scoring runs:

1. Calibrate Gemini 336L RGB intrinsics and write `config/camera_intrinsics.yaml`.
2. Measure target outer rectangle size and write `config/target_geometry.yaml`.
3. Measure or tune laser origin relative to the camera frame and write
   `config/laser_extrinsic.yaml`.

Camera coordinate convention:

- `x`: image right
- `y`: image down
- `z`: camera forward

The laser compensation computes:

```text
direction = target_center_in_camera - laser_origin_in_camera
yaw_delta = atan2(direction.x, direction.z)
pitch_delta = atan2(-direction.y, hypot(direction.x, direction.z))
```

