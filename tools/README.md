# Tools

Command-line utilities live here.

- `record_dataset`: capture raw RGB/depth frames and metadata.
- `replay_dataset`: replay recorded frames through the same pipeline.
- `calibrate_camera`: generate `config/camera_intrinsics.yaml`.
- `calibrate_laser`: tune `config/laser_extrinsic.yaml`.
- `serial_monitor`: inspect packets sent to the gimbal.

The first version exposes these workflows through the `apps/` executables.

