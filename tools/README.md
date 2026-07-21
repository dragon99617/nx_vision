# Tools

Command-line utilities live here.

- `record_dataset`: capture raw RGB/depth frames and metadata.
- `replay_dataset`: replay recorded frames through the same pipeline.
- `calibrate_camera`: generate `config/camera_intrinsics.yaml`.
- `calibrate_laser`: tune `config/laser_extrinsic.yaml`.
- `serial_monitor`: inspect packets sent to the gimbal.

The first version exposes these workflows through the `apps/` executables.

Implemented standalone checks:

- `validate_camera_imu_extrinsic.py` rejects non-orthonormal camera/body
  rotations and prints the installed axis mapping.
- `fit_gimbal_feedforward.py` performs a robust held-out fit of inertia,
  viscous friction, Coulomb friction, and the optional pitch gravity term from
  a 1 kHz identification CSV. A numerical pass is not permission to enable
  torque feedforward; replay and hardware-limit checks remain mandatory.

