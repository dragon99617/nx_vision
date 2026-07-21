# Calibration

Required before real scoring runs:

1. Calibrate Gemini 336L RGB intrinsics and write `config/camera_intrinsics.yaml`.
2. Measure target outer rectangle size and write `config/target_geometry.yaml`.
3. Measure or tune laser origin relative to the camera frame and write
   `config/laser_extrinsic.yaml`.
4. Measure camera-to-gimbal-body rotation and write the 3x3
   `body_from_camera` matrix in `config/control.yaml`. Run
   `python3 tools/validate_camera_imu_extrinsic.py config/control.yaml`, then
   rotate yaw and pitch separately while viewing one fixed target. The recovered
   world angle must stay constant and the reported camera-axis signs must match
   the installation.
5. Tune `camera_timestamp_phase_offset_us` only from timestamped motion data;
   do not use it to hide an incorrect rotation matrix. Keep precise timestamp
   mode enabled for acceptance tests.

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

