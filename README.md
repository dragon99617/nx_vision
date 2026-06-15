# nx_vision

Orin NX vision stack for a self-aiming gimbal task. The project captures RGB
frames, detects the target, estimates pose with PnP, compensates for the laser
origin, and sends relative yaw/pitch angle errors to an STM32 gimbal controller.

The first implementation is OpenCV based. The camera layer is isolated so native
Orbbec SDK depth capture can be added later without rewriting the PnP, task,
serial, pipeline, or debug-view layers.

## Features

- OpenCV RGB capture and processing pipeline.
- Target rectangle detection, preprocessing, tracking, and laser detection.
- PnP pose solving and laser-origin compensation.
- Multiple task modes for the basic and advanced requirements.
- Serial packet generation for gimbal integration.
- Debug Studio views for raw frames, masks, contours, pose, and serial output.
- YAML-based configuration for camera, target geometry, serial, tasks, and debug
  layout.

## Repository Layout

```text
apps/        Executable entry points for runtime, debug, replay, calibration, and tasks.
config/      YAML configuration files used by the applications.
data/        Runtime data notes and expected data directories.
docs/        Integration, protocol, calibration, debug, and coordinate-frame notes.
include/     Public C++ headers.
scripts/     Jetson dependency installation, build, run, and serial setup helpers.
src/         Core camera, vision, pose, task, pipeline, communication, and debug code.
tests/       Lightweight C++ tests.
tools/       Developer support notes and utilities.
```

## Requirements

- CMake 3.16 or newer.
- C++17 compiler.
- OpenCV development packages with `core`, `imgproc`, `highgui`, `calib3d`,
  `videoio`, and `imgcodecs`.
- POSIX shell environment for the helper scripts.
- Orbbec SDK v2 is optional for future native depth capture.

On Jetson, install the base dependencies with:

```bash
./scripts/install_deps_jetson.sh
```

## Build

```bash
./scripts/build_release.sh
```

Equivalent manual command:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## Run

Start the interactive debug view:

```bash
./scripts/run_debug_studio.sh
```

Start the runtime pipeline:

```bash
./scripts/run_runtime.sh
```

Both scripts pass `config/` to the executable. Update the YAML files in
`config/` before running on new hardware.

## Tests

Tests are enabled by default through `NXVISION_BUILD_TESTS`.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## Gimbal Protocol

The runtime sends relative angle error to the gimbal:

```text
A,yaw_cdeg,pitch_cdeg,dist_mm,valid\n
```

- `yaw_cdeg`: relative yaw angle error in `0.01 deg`.
- `pitch_cdeg`: relative pitch angle error in `0.01 deg`.
- `dist_mm`: laser-origin-to-target distance estimate.
- `valid`: `1` when a target is valid, otherwise `0`.

The STM32 side should apply the packet as:

```text
yaw_target = current_yaw + yaw_delta
pitch_target = current_pitch + pitch_delta
```

## Documentation

- `docs/protocol.md`: serial packet format and gimbal integration notes.
- `docs/yuntai_integration.md`: gimbal-side integration checklist.
- `docs/calibration.md`: camera and target calibration notes.
- `docs/coordinate_frames.md`: coordinate-frame conventions.
- `docs/debug_studio.md`: Debug Studio panels and shortcuts.
- `docs/topic_summary.md`: task-to-requirement mapping.
